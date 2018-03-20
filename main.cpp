// IO
#include <iostream>
#include <fstream>
#include <iomanip>
#include "Serial.h"

// GPS
#include "nmea.h"

// USRP
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

// FFT
#include <fftw3.h>

// Network
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

void get_channel_data(uhd::usrp::multi_usrp::sptr usrp, double freq, unsigned long num_samps, std::vector<std::complex<short>>* buff)
{
    uhd::tune_request_t tune_request(freq, 400000);
    usrp->set_rx_freq(tune_request);

    //create a receive streamer
    uhd::stream_args_t stream_args("sc16","sc16");
    std::vector<size_t> channel_nums;
    channel_nums.push_back(boost::lexical_cast<size_t>(0));
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    //setup streaming
    uhd::stream_cmd_t stream_cmd( uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE );
    stream_cmd.num_samps = size_t(num_samps);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    rx_stream->issue_stream_cmd(stream_cmd);

    uhd::rx_metadata_t md;

    size_t num_rx_samps = rx_stream->recv(&buff->front(), buff->size(), md, 3.0, false);
}

void measure_signal(std::vector<std::complex<short>>* buff, int freq, double fs, double &signal, double &noise)
{
    // F = 2**int(numpy.ceil(numpy.log2(len(sig))))
    //auto F = int(pow(2, ceil(log2(buff->size()))));
    auto F = buff->size();

    // sig_fft = 20.0*numpy.log10(numpy.abs(numpy.fft.fftshift(numpy.fft.fft(sig, F))))
    fftw_complex sig[buff->size()];
    fftw_complex result[F];

    for(unsigned long ii = 0; ii < buff->size(); ii++)
    {
        sig[ii][0] = buff->at(ii).real();
        sig[ii][1] = buff->at(ii).imag();
    }

    fftw_plan plan = fftw_plan_dft_1d(F,
                                      sig,
                                      result,
                                      FFTW_FORWARD,
                                      FFTW_ESTIMATE);

    fftw_execute(plan);
    fftw_destroy_plan(plan);

    // 20.0 * abs(log10(fft_sig))
    std::vector<double> sig_fft(F);
    for(unsigned long ii = 0; ii < F; ii++)
    {
        sig_fft.at(ii) = 20.0 * std::log10(std::abs(std::complex<double>(result[ii][0], result[ii][1])));
    }

    // f_bins = numpy.fft.fftshift(numpy.fft.fftfreq(F, 1.0/fs))
    auto sig_bw_idx = int(std::ceil(75000.0 / fs * F));
    auto noise_bw_idx = int(std::ceil(100000.0 / fs * F));


    // sig_mask = numpy.bitwise_and(f_bins > -75000, f_bins < 75000)
    // sig_mean = numpy.mean(sig_fft[sig_mask])
    // noise_mean = numpy.mean(sig_fft[numpy.logical_not(sig_mask)])

    double sig_sum = 0.0;
    double sig_num = 0.0;
    double noise_sum = 0.0;
    double noise_num = 0.0;
    for(unsigned long ii = 0; ii < F; ii++)
    {
        if(ii < sig_bw_idx || ii > F-sig_bw_idx)
        {
            sig_sum += sig_fft.at(ii);
            sig_num++;
        }
        else if(ii < noise_bw_idx || ii > F-noise_bw_idx)
        {
            noise_sum += sig_fft.at(ii);
            noise_num++;
        }
    }

    signal = sig_sum / sig_num;
    noise = noise_sum / noise_num;

    //std::cout << freq << "   " << signal << "   " << noise << std::endl;
}

void write_csv(double freq, double signal, double noise, std::vector<std::complex<short>>* buff)
{
    std::stringstream ss;
    ss << "test_" << int(freq) << ".csv";

    std::ofstream myfile;
    myfile.open( ss.str() );

    for(const auto& it : *buff)
    {
        myfile << it.real() << "," << it.imag() << std::endl;
    }

    std::ofstream myfile2;
    myfile2.open( "summary.csv", std::ios_base::app );
    myfile2 << freq << "," << signal << "," << noise << "," << (signal-noise) << std::endl;
}

void broadcast_results(int sd, struct sockaddr_in* Remote_Address, double freq, double signal, double noise, double lat, double lon)
{
    std::stringstream ss;
    ss << std::setprecision(9) << freq << "," << signal << "," << noise << "," << lat << "," << lon;

    ssize_t rc = \
        sendto(sd, ss.str().data(), ss.str().size(), 0, (struct sockaddr *) Remote_Address, sizeof(*Remote_Address));

    if( rc < 0 )
    {
        printf("cannot send data\n");
        close(sd);
    }
}

bool get_gps_loc(Serial &serial_device, double &lat, double &lon)
{
    std::string line = serial_device.readline();

    if(get_lat_lon(line, lat, lon))
    {
        //std::cout << line << "  " << lat << "  " << lon << std::endl;

        return true;
    }

    return false;
}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    uhd::set_thread_priority_safe();

    std::string device_args("serial=30C7DF8");
    std::string ant("RX2");
    std::string ref("internal");

    double fs(500000);
    double gain(0);

    // USRP object
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(device_args);

    usrp->set_clock_source(ref);
    usrp->set_rx_rate(fs);
    usrp->set_rx_gain(gain);
    usrp->set_rx_bandwidth(fs);
    usrp->set_rx_antenna(ant);


    // socket creation
    int sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if( sd < 0 )
    {
        printf("cannot open socket\n");
        return EXIT_FAILURE;
    }

    int broadcast = 1;
    if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast) == -1)
    {
        perror("setsockopt (SO_BROADCAST)");
        return EXIT_FAILURE;
    }

    struct sockaddr_in Remote_Address;
    struct hostent *hostPointer;

    /*Fill in client's sockaddr_in */
    bzero(&Remote_Address, sizeof(Remote_Address));
    Remote_Address.sin_family=AF_INET;
    hostPointer=gethostbyname("192.168.1.255");
    memcpy((unsigned char * ) &Remote_Address.sin_addr, (unsigned char *) hostPointer -> h_addr, hostPointer -> h_length);
    Remote_Address.sin_port=htons(9999);

    // Create buffer
    std::vector<std::complex<short>> buff(50000);
    double noise, signal;
    double lat, lon;

    // Open serial device
    Serial serial_device("/dev/ttyUSB0", 4800);

    // =================================================================================================================
    // Main loop over channels
    // =================================================================================================================
    while(true)
    {
        for (int freq = 88100000; freq <= 107900000; freq += 200000)
        {
            get_channel_data(usrp, double(freq), buff.size(), &buff);
            get_gps_loc(serial_device, lat, lon);
            measure_signal(&buff, freq, fs, signal, noise);
            //write_csv(freq, signal, noise, &buff);
            broadcast_results(sd, &Remote_Address, freq, signal, noise, lat, lon);
        }
    }

    return EXIT_SUCCESS;
}

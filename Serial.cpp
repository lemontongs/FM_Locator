//
// Created by mlamonta on 3/18/18.
//

#include "Serial.h"

#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <cstring>

Serial::Serial(std::string port, int baud)
{
    sf = open( port.data(), O_RDWR| O_NOCTTY );

    /* Error Handling */
    if ( sf < 0 )
    {
        std::cout << "Error " << errno << " from open: " << strerror(errno) << std::endl;
        return;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( sf, &tty ) != 0 )
    {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return;
    }

    /* Set Baud Rate */
    switch (baud)
    {
        case 4800:
            cfsetospeed (&tty, (speed_t)B4800);
            cfsetispeed (&tty, (speed_t)B4800);
            break;

        case 9600:
            cfsetospeed (&tty, (speed_t)B9600);
            cfsetispeed (&tty, (speed_t)B9600);
            break;

        case 115200:
            cfsetospeed (&tty, (speed_t)B115200);
            cfsetispeed (&tty, (speed_t)B115200);
            break;

        default:
            std::cout << "Error: Unsupported baudrate specified: " << baud << std::endl;
            return;
    }

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( sf, TCIFLUSH );
    if ( tcsetattr ( sf, TCSANOW, &tty ) != 0)
    {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return;
    }

    initialized = true;
}

std::string Serial::readline()
{
    if(!initialized)
    {
        return std::string();
    }

    ssize_t n = 0;
    char buf = '\0';

    std::stringstream ss;

    do
    {
        n = read( sf, &buf, 1 );
        if (buf != '\r' && buf != '\n')
            ss << buf;
    }
    while( buf != '\r' && n > 0);

    if (n < 0)
    {
        std::cout << "Error reading: " << strerror(errno) << std::endl;
    }
    else if (n == 0)
    {
        std::cout << "Read nothing!" << std::endl;
    }

    return ss.str();
}

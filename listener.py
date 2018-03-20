import copy
import socket
from matplotlib import pyplot
from matplotlib import animation
import serial
from pynmea import nmea
import pyproj
import threading

exit_requested = False

port = 9999
address = "192.168.1.255"
interface = "0.0.0.0"

# Shared data
dat_lock = threading.Lock()
database = {}

# Shared data
loc_lock = threading.Lock()
latest_location = {
    "timestamp": None,
    "latitude": None,
    "lat_direction": None,
    "longitude": None,
    "lon_direction": None,
    "gps_qual": None,
    "num_sats": None,
    "horizontal_dil": None,
    "antenna_altitude": None,
    "altitude_units": None,
    "geo_sep": None,
    "geo_sep_units": None,
    "age_gps_data": None,
    "ref_station_id": None,
    "x": None,
    "y": None
}


def get_location():
    global exit_requested

    s_port = serial.Serial(port="/dev/ttyUSB0", baudrate=4800, timeout=1)
    gpgga = nmea.GPGGA()

    from pprint import pprint

    while not exit_requested:
        data = s_port.read_until()
        if data.startswith('$GPGGA'):
            gpgga.parse(data)
            loc_lock.acquire()
            latest_location["timestamp"] = gpgga.timestamp
            latest_location["latitude"] = gpgga.latitude
            latest_location["lat_direction"] = gpgga.lat_direction
            latest_location["longitude"] = gpgga.longitude
            latest_location["lon_direction"] = gpgga.lon_direction
            latest_location["gps_qual"] = gpgga.gps_qual
            latest_location["num_sats"] = gpgga.num_sats
            latest_location["horizontal_dil"] = gpgga.horizontal_dil
            latest_location["antenna_altitude"] = gpgga.antenna_altitude
            latest_location["altitude_units"] = gpgga.altitude_units
            latest_location["geo_sep"] = gpgga.geo_sep
            latest_location["geo_sep_units"] = gpgga.geo_sep_units
            latest_location["age_gps_data"] = gpgga.age_gps_data
            latest_location["ref_station_id"] = gpgga.ref_station_id

            def str_latlon_to_decimal_deg(s_lat, s_lon):
                return (float(s_lat[:2]) + float(s_lat[2:]) / 60., float(s_lon[:3]) + float(s_lon[3:]) / 60.)

            p = pyproj.Proj(init='EPSG:3857')
            lat_deg, lon_deg = str_latlon_to_decimal_deg(gpgga.latitude, gpgga.longitude)
            latest_location["x"], latest_location["y"] = p(lon_deg, lat_deg)

            pprint(latest_location)

            loc_lock.release()


def get_data():
    global exit_requested

    # Instantiate a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Allow address reuse
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Allow listening on broadcast addresses
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Bind to the port
    sock.bind(('', port))

    while not exit_requested:
        # print "waiting for message"
        (msg, _) = sock.recvfrom(1024)
        # print msg

        fields = msg.split(",")

        if len(fields) == 3:
            freq = str(int(float(fields[0])))
            signal = float(fields[1])
            noise = float(fields[2])

            snr = signal-noise

            print freq, signal, noise

            # Get the current location
            loc_lock.acquire()
            loc = copy.deepcopy(latest_location)
            loc_lock.release()

            # Update the database
            dat_lock.acquire()
            if freq not in database:
                database[freq] = 0.0
            database[freq] = (float(freq), snr, loc)
            dat_lock.release()


fig = pyplot.figure()
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)


def plot_data(_):
    dat_lock.acquire()
    left = [f for (x, (f, s, l)) in database.items()]
    values = [s for (x, (f, s, l)) in database.items()]
    xs = [l['x'] for (x, (f, s, l)) in database.items()]
    ys = [l['y'] for (x, (f, s, l)) in database.items()]
    dat_lock.release()

    # Clear the plots
    ax1.clear()
    ax2.clear()

    # Plot 1
    rects = ax1.bar(left, values, width=200000)

    for ii, rect in enumerate(rects):
        height = rect.get_height()

        if height > 1.0:
            width = rect.get_width()
            x = rect.get_x()
            xc = x + width / 2.
            ax1.text(xc, 1.05 * height, '%1.1f' % (xc/1e6), ha='center', va='bottom')

    # Plot 2
    ax2.plot(xs, ys, '.')


# Location Thread
t_loc = threading.Thread(target=get_location)
t_loc.daemon = True
t_loc.start()

# Data Thread
t_dat = threading.Thread(target=get_data)
t_dat.daemon = True
t_dat.start()

# Plot Thread
ani = animation.FuncAnimation(fig, plot_data)
pyplot.show()

exit_requested = True
t_loc.join()
t_dat.join()

print "DONE"

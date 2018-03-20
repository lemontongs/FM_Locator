//
// Created by mlamonta on 3/18/18.
//

#ifndef FM_LOCATOR_NMEA_H
#define FM_LOCATOR_NMEA_H

#include <string>

bool validate_checksum(std::string &line);
bool get_lat_lon(std::string &line, double &lat, double &lon);

#endif //FM_LOCATOR_NMEA_H

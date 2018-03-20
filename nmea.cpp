//
// Created by mlamonta on 3/18/18.
//

#include "nmea.h"

#include <iostream>
#include <sstream>
#include <vector>

bool validate_checksum(std::string &line)
{
    int pos = line.find("*");

    if( pos == std::string::npos )
    {
        return false;
    }
    else
    {
        char cksum = 0;
        for(int ii = 1; ii < pos; ii++)
        {
            cksum ^= line[ii];
        }

        unsigned int x;
        std::stringstream ss;
        ss << std::hex << line.substr(pos+1,pos+3);
        ss >> x;

        return int(cksum) == x;
    }
}

bool get_lat_lon(std::string &line, double &lat, double &lon)
{
    if( (0 == line.find("$GPGGA")) && validate_checksum(line) )
    {
        std::vector<std::string> parts;

        int pos = 0;
        int l_pos = 0;
        while(pos != std::string::npos)
        {
            pos = line.find(",", pos+1);
            parts.push_back(line.substr(l_pos+1, pos - l_pos - 1));
            l_pos = pos;
        }

        std::string lat_s = parts.at(2);
        std::string lat_d = parts.at(3);
        std::string lon_s = parts.at(4);
        std::string lon_d = parts.at(5);

        double lat_mod = 1.0;
        if(lat_d.find("N") == std::string::npos)
            lat_mod = -1.0;

        double lon_mod = 1.0;
        if(lon_d.find("E") == std::string::npos)
            lon_mod = -1.0;

        lat = lat_mod * (std::stof(lat_s.substr(0, 2)) + std::stof(lat_s.substr(2, lat_s.length())) / 60.0);
        lon = lon_mod * (std::stof(lon_s.substr(0, 3)) + std::stof(lon_s.substr(3, lon_s.length())) / 60.0);

        return true;
    }
    else
    {
        return false;
    }
}

/*
int main()
{
    std::string line = "$GPGGA,233440.000,4258.9798,N,07129.3574,W,1,08,1.3,95.2,M,-33.2,M,,0000*5B";

    double lat, lon;

    if(get_lat_lon(line, lat, lon))
    {
        std::cout << lat << "  " << lon << std::endl;
    }
}
*/

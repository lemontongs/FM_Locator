//
// Created by mlamonta on 3/18/18.
//

#ifndef FM_LOCATOR_SERIAL_H
#define FM_LOCATOR_SERIAL_H

#include <string>

class Serial
{
public:
    Serial(std::string port, int baud);
    std::string readline();

 private:
    int sf;
    bool initialized = false;
};

#endif //FM_LOCATOR_SERIAL_H

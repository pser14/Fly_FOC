#ifndef _AS5600_FOC_HPP
#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_ANGLE_REG 0x0E

void Init_AS5600();
float readAngle();

#define _AS5600_FOC_H
#endif
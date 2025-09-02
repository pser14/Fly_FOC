#ifndef CURRENT_H
#include "Arduino.h"

int pinA;
int pinB;
int pinC;
float offsetA;    
float offsetB;    
float offsetC;
float shunt_resistance ; //分流电阻阻值，单位欧姆
float current_sense_gain;  //电流传感器增益
float current_A;
float current_B;
float current_C;
float volts_to_amps; //电压转电流的转换系数
float gainA;
float gainB;
float gainC;

float readADCVoltage(const int pin);
void ADCInit(const int pinA,const int pinB,const int pinC);
void calibrateOffset();
float getPhaseCurrent();

#define CURRENT_H
#endif
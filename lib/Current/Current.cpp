#include "Arduino.h"
#include "Current.h"

#define _ADC_VOLTAGE_REF 3.3  // ADC参考电压
#define _ADC_RESOLUTION 4095.0f // 12位ADC分辨率

#define _SAMPLES 100  // 采样次数
#define _ADC_CONVERT_VOLTAGE (_ADC_VOLTAGE_REF / _ADC_RESOLUTION) // ADC值转电压

float shunt_resistance = 0.01; //分流电阻阻值，单位欧姆
float current_sense_gain = 50;  //电流传感器增益

void ADCInit(int pinA,int pinB,int pinC)
{
    pinA = pinA;
    pinB = pinB;
    pinC = pinC;
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
    calibrateOffset();
}

void calibrateOffset()
{
    offsetA = 0;
    offsetB = 0;
    offsetC = 0;

    for (int i = 0; i < _SAMPLES; i++) {
        offsetA += readADCVoltage(pinA);
        offsetB += readADCVoltage(pinB);
        offsetC += readADCVoltage(pinC);
        delay(1);
    }
    offsetA = offsetA / (float)_SAMPLES;
    offsetB = offsetB / (float)_SAMPLES;
    offsetC = offsetC / (float)_SAMPLES;
}

float readADCVoltage(const int pin)
{
    uint32_t raw = analogRead(pin);
    float voltage = raw * _ADC_CONVERT_VOLTAGE;
    return voltage;
}

float getPhaseCurrent()
{
    float voltageA = readADCVoltage(pinA) - offsetA;
    float voltageB = readADCVoltage(pinB) - offsetB;
    float voltageC = readADCVoltage(pinC) - offsetC;

    volts_to_amps = 1.0f / (shunt_resistance * current_sense_gain);

    gainA = volts_to_amps * -1;
    gainB = volts_to_amps * -1;
    gainC = volts_to_amps;

    current_A = voltageA * gainA;
    current_B = voltageB * gainB;
    current_C = voltageC * gainC;

    return current_A,current_B,current_C;
}
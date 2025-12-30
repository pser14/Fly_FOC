#include "Wire.h" 
#include "AS5600_foc.hpp"
#include <Arduino.h> 

#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_ANGLE_REG 0x0E

void Init_AS5600()
{
  Serial.begin(115200);
  Wire.begin(23,5);

  Wire.beginTransmission(AS5600_ADDRESS);//检查AS5600是否连接
  byte error = Wire.endTransmission();

    if (error == 0) {
    Serial.println("AS5600 found!");
  } else {
    Serial.println("AS5600 not found. Check connections.");
    while(1); // 停止程序
  }

}

float readAngle()
{
  // 读取原始角度（12位，0-4095对应0-360度）
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_RAW_ANGLE_REG); // 请求原始角度寄存器
  Wire.endTransmission(false); // 保持连接
  
  Wire.requestFrom(AS5600_ADDRESS, 2); // 请求2字节数据
  while (Wire.available() < 2); // 等待数据
  
  byte highByte = Wire.read();
  byte lowByte = Wire.read();

  //合并两个字节的数据
  uint16_t rawAngle = (highByte << 8) | lowByte;
  
  // 转换为角度（0-360度）
  float angle = (rawAngle * 360.0) / 4096.0;
  
  return angle;
}
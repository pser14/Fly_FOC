#ifndef FLY_FOC_HPP
#define FLY_FOC_HPP

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/timer.h>

extern float motor_target;
extern int commaPosition;
extern float Kp, Ki, Kd;

//电机参数结构体
typedef struct motor_data{ 
  float voltage_power_supply;
  float Ualpha;
  float Ubeta;
  float Ua;
  float Ub;
  float Uc;
  float dc_a;
  float dc_b;
  float dc_c;
}motor_data_t;

//速度PID参数结构体
typedef struct velocity_pid{
  float Kp;                                         
  float Ki;
  float Kd;
  float integral;
  float prev_error;
  float output_limit;
}velocity_pid_t;

//角度参数结构体
typedef struct control_data{
  float target_angle;                  //目标角度
  float current_angle;                 //当前角度
  float target_velocity;               //目标速度
  float current_velocity;              //当前速度
  bool  enable;                        //电机使能状态
}control_data_t;



//全局声明参数结构体
extern velocity_pid_t velocity_pid;
extern control_data_t control_data;
extern motor_data_t motor_data;

// 函数声明
void Motor_init(int OUTPUTA, int OUTPUTB, int OUTPUTC, int ENABLE_PIN);
void OutputValtage(float Uq, float Ud, float angle_el);
void calibrateMotor();
float Normalization(float angle);
float EleAngle(float shaft_angle, int pole_pairs);
float lowPassFillter(float input, float prev_output, float alpha);
float pidController(float error,float dt);
float velocityPidcontroller(float error,float dt);
String serialReceiveUserCommand();
void processCommand(String cmd);
void handleSerialCommands();
float angleDifference(float target, float current);
float IRAM_ATTR AS5600_ReadRawAngle();

#endif
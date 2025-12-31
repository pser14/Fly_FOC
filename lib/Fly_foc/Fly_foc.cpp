#include "Fly_foc.hpp"
#include "AS5600_foc.hpp"

#define _3PI_2 4.71238898038f

int pwmA = 32;
int pwmB = 33;
int pwmC = 25;
float zero_electric_angle = 0;
float Kp = 2, Ki = 0, Kd = 0;
float intergral = 0,error_prev = 0;
float current_angel_input = 0;

velocity_pid_t velocity_pid = {0.5,0.2,0.01,0,0,6};
control_data_t control_data = {0,0,-200,0,false};
motor_data_t motor_data ={12.0,0,0,0,0,0,0,0,0};

void Motor_init(int OUTPUTA,int OUTPUTB,int OUTPUTC,int ENABLE_PIN)
{
    Serial.begin(115200);
    Serial.println("开始电机初始化");

    ENABLE_PIN = ENABLE_PIN;
    OUTPUTA = OUTPUTA;
    OUTPUTB = OUTPUTB;
    OUTPUTC = OUTPUTC;
    //设置pwm口输出模式
    pinMode(ENABLE_PIN,OUTPUT);
    digitalWrite(ENABLE_PIN,HIGH);
    pinMode(OUTPUTA, OUTPUT);
    pinMode(OUTPUTB, OUTPUT);
    pinMode(OUTPUTC, OUTPUT);
    //对电机进行使能
    pinMode(ENABLE_PIN,OUTPUT);
    digitalWrite(ENABLE_PIN,HIGH);
    //将pwm绑定到电机的引脚上
    ledcAttachPin(OUTPUTA,0);
    ledcAttachPin(OUTPUTB,1);
    ledcAttachPin(OUTPUTC,2);
    //设置pwm的频率和精度，这里为30kHz，8bit
    ledcSetup(0, 30000, 8);
    ledcSetup(1, 30000, 8);
    ledcSetup(2, 30000, 8);

    Serial.println("完成PWM初始化设置");
    delay(3000);

}

void OutputValtage(float Uq,float Ud, float angle_el)
{
    //对输入的电角度进行归一化。
    angle_el = Normalization(angle_el + zero_electric_angle);
    // 帕克逆变换
    motor_data.Ualpha =  -Uq*sin(angle_el); 
    motor_data.Ubeta =   Uq*cos(angle_el); 

    // 克拉克逆变换
    motor_data.Ua = motor_data.Ualpha + motor_data.voltage_power_supply/2;
    motor_data.Ub = (sqrt(3)*motor_data.Ubeta-motor_data.Ualpha)/2 + motor_data.voltage_power_supply/2;
    motor_data.Uc = (-motor_data.Ualpha-sqrt(3)*motor_data.Ubeta)/2 + motor_data.voltage_power_supply/2;

    //对电机输入的电压进行限制
    motor_data.dc_a = _constrain(motor_data.Ua / motor_data.voltage_power_supply, 0.0f , 1.0f );
    motor_data.dc_b = _constrain(motor_data.Ub / motor_data.voltage_power_supply, 0.0f , 1.0f );
    motor_data.dc_c = _constrain(motor_data.Uc / motor_data.voltage_power_supply, 0.0f , 1.0f );

    //写入PWM到PWM 0 1 2 通道
    ledcWrite(0, motor_data.dc_a*255);
    ledcWrite(1, motor_data.dc_b*255);
    ledcWrite(2, motor_data.dc_c*255);
}

float Normalization(float shaft_angle)
{   
    float a = fmod(shaft_angle, 2*PI);
    shaft_angle = a >= 0 ? a : (a + 2*PI); 
    return shaft_angle;
}

float EleAngle(float shaft_angle,int pole_pairs)
{
    float eletric_angle = shaft_angle * pole_pairs;
    return eletric_angle*PI/180;
}

String serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {
      
      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');//检测字符串中的逗号
      if(commaPosition != -1)//如果有逗号存在就向下执行
      {
          motor_target = command.substring(0,commaPosition).toDouble();            //电机角度
          Serial.println(motor_target);
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}

//电机零点校准
void calibrateMotor() {
  Serial.println("开始电机校准...");

  float align_voltage = 3.0;                       // 施加3V的对齐电压

  OutputValtage(align_voltage, 0,_3PI_2);           // 对齐到0度位置
  delay(1000);                                      // 等待电机稳定                     

  // `readAngle()` returns degrees (0..360). `EleAngle()` expects degrees and
  // converts to radians internally, so do NOT run `Normalization()` (which
  // expects radians) on the degree value — that produced incorrect offsets.
  float aligned_shaft_angle = readAngle(); // 读取当前角度（度）作为对齐角度
  zero_electric_angle = -EleAngle(aligned_shaft_angle, 7); // 计算电气零点偏移（弧度）

  Serial.println("校准完成");
  Serial.println("电机零点角度: " + String(zero_electric_angle * 180 / (7 * PI)) + "度");

}

float pidController(float error,float dt) {

  float proportional = Kp * error;

  intergral += (dt/2) * (error + error_prev);
  intergral = constrain(intergral, -6, 6);

  // float output = Kp*error + Ki*intergral;
  float output = Kp*error;

  error_prev = error;

  return output;
}

float velocityPidcontroller(float error,float dt){
  float proportional = velocity_pid.Kp * error;
  velocity_pid.integral += error * dt;

  velocity_pid.integral = constrain(velocity_pid.integral,
                                   -velocity_pid.output_limit/velocity_pid.Ki,
                                    velocity_pid.output_limit/velocity_pid.Ki);

  float intergral = velocity_pid.Ki * velocity_pid.integral;

  float derivative = velocity_pid.Kd * (error - velocity_pid.prev_error) / dt;
  velocity_pid.prev_error = error;
  //对输出进行限制，大小约为±6伏
  float output = proportional;
  output = constrain(output,-velocity_pid.output_limit,velocity_pid.output_limit);
  return output;

}

float angleDifference(float target, float current) {
    float error = target - current;  // 原始误差
    // 将误差规范到[-180, 180]度范围内
    if (error > 180) error -= 360;   // 正向超限调整
    else if (error < -180) error += 360; // 负向超限调整
    return error;
}

float lowPassFilter(float input, float prev_output, float alpha){
  float velocity = alpha * input + (1 - alpha) * prev_output;
  return velocity;
}


//开环速度函数
float velocityOpenloop(float target_velocity){
  unsigned long now_us = micros();  //获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值
  float shaft_angle = 0;
  float open_loop_timestamp = 0;

  //计算当前每个Loop的运行时间间隔
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = Normalization(shaft_angle + target_velocity*Ts);
  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  float Uq = 12/3;

  
  // OutputValtage(Uq,  0, -EleAngle(shaft_angle, 7));
  
  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return shaft_angle;
}

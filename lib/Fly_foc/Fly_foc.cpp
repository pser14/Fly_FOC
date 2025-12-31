#include "Fly_foc.hpp"
#include "AS5600_foc.hpp"

#define _3PI_2 4.71238898038f

int pwmA = 32;
int pwmB = 33;
int pwmC = 25;
float zero_electric_angle = 0;
float Kp = 0.03, Ki = 0.1, Kd = 0;
float intergral = 0,error_prev = 0;
float current_angel_input = 0;

velocity_pid_t velocity_pid = {0.5,0.2,0.01,0,0,6};
control_data_t control_data = {0,0,-200,0,false,false,false};
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

float calibratedEleAngle(float shaft_angle, int pole_pairs) {
    float elec_angle = EleAngle(shaft_angle, pole_pairs) - zero_electric_angle;
    return fmod(elec_angle, 2 * PI);
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
    Serial.println("请确保电机轴可以自由旋转！");

    const float align_voltage = 3.0f;      // 对齐电压
    const uint32_t align_time_ms = 1000;   // 对齐时间
    const int samples = 100;               // 采样点数
    float angle_sum = 0.0f;
    int valid_samples = 0;
    
    // 先读取初始角度（未对齐时）
    float initial_angle = readAngle();
    Serial.printf("初始角度: %.2f度\n", initial_angle);
    
    // 对齐到特定方向（_3PI_2 = 270度）
    Serial.println("施加对齐电压...");
    OutputValtage(align_voltage, 0, _3PI_2);
    delay(align_time_ms);
    
    // 多次采样求平均，提高精度
    Serial.println("采样对齐角度...");
    for (int i = 0; i < samples; i++) {
        float angle = readAngle();  // 使用安全的读取函数
        
        if (!isnan(angle)) {
            angle_sum += angle;
            valid_samples++;
        }
        
        delay(2);  // 采样间隔
    }
    
    // 停止输出，让电机自由
    OutputValtage(0, 0, 0);
    delay(500);
    
    if (valid_samples > 0) {
        float aligned_shaft_angle = angle_sum / valid_samples;  // 平均机械角度（度）
        
        // 计算电气零点偏移
        // 对齐时的电气角度应该是_3PI_2（270度）
        // 实际读取的机械角度对应的电气角度是：EleAngle(aligned_shaft_angle, pole_pairs)
        // 零点偏移 = 期望电气角度 - 实际电气角度
        zero_electric_angle = _3PI_2 - EleAngle(aligned_shaft_angle, 7);
        
        // 规范化到0-2π范围内
        while (zero_electric_angle > 2 * PI) {
            zero_electric_angle -= 2 * PI;
        }
        while (zero_electric_angle < 0) {
            zero_electric_angle += 2 * PI;
        }
        
        Serial.println("校准完成");
        Serial.printf("对齐后平均角度: %.2f度\n", aligned_shaft_angle);
        Serial.printf("电气零点偏移: %.4f rad (%.2f度)\n", 
                     zero_electric_angle, zero_electric_angle * 180.0f / PI);
        
        // 验证校准结果
        Serial.println("验证校准结果...");
        delay(200);
        
        float test_angle = readAngle();
        if (!isnan(test_angle)) {
            float elec_angle = EleAngle(test_angle, 7) + zero_electric_angle;
            elec_angle = fmod(elec_angle, 2 * PI);
            Serial.printf("当前机械角度: %.2f度, 校准后电气角度: %.2f度\n",
                         test_angle, elec_angle * 180.0f / PI);
        }
    } else {
        Serial.println("校准失败：无法读取有效角度数据！");
    }
    
    Serial.println("校准流程结束");
}

float positionPidController(float error,float dt) {

  float proportional = Kp * error;

  intergral += (dt/2) * (error + error_prev);
  intergral = constrain(intergral, -6, 6);

  float output = Kp*error + Ki*intergral;

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


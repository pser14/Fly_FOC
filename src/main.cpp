#include "Fly_foc.hpp"
#include "AS5600_foc.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/timer.h>

#define PWMA 32
#define PWMB 33
#define PWMC 25
#define ENABLE_PIN 12

int pole_pairs = 7;
// open-loop mapping gain (Uq = openloop_gain * target_velocity)
float openloop_gain = 0.5f;

TaskHandle_t controlTaskHandle = NULL;    // 电机控制任务句柄
TaskHandle_t serialTaskHandle = NULL;     // 串口通信任务句柄
TaskHandle_t velocityTaskHandle = NULL;   // 速度计算任务句柄
SemaphoreHandle_t i2cMutex = NULL; 
SemaphoreHandle_t dataMutex = NULL;        //用于数据保护的互斥锁

//安全读取i2c霍尔角度传感器数据，避免线程被阻塞
float readAngleSafe(){
  if(xSemaphoreTake(i2cMutex,pdMS_TO_TICKS(10)) == pdTRUE){
    float angle = readAngle();
    xSemaphoreGive(i2cMutex);
    return angle;
  }
  return NAN;
}

//角速度计算任务
void velocityTask(void *pvParameters){
  TickType_t xLastWakeTIme = xTaskGetTickCount();
  float previous_angle = 0;
  uint32_t previous_time = 0;
  float filtered_velocity = 0;
  const float filter_alpha = 0.4;

  while (1){
    float current_angle = readAngleSafe();
    uint32_t current_time = micros();

    if(previous_time > 0 && !isnan(current_angle)){
      uint32_t delta_time = current_time - previous_time;
      if(delta_time > 0){
        float angle_diff = angleDifference(current_angle,previous_angle);
        float raw_velocity = (angle_diff * 1000000.0f) / delta_time;
        filtered_velocity = lowPassFilter(raw_velocity,filtered_velocity,filter_alpha);

        if(xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
          control_data.current_velocity = filtered_velocity;
          control_data.current_angle =current_angle;
          xSemaphoreGive(dataMutex);
        }
      }
    }
    previous_angle = current_angle;
    previous_time = current_time;
    
    vTaskDelayUntil(&xLastWakeTIme,pdMS_TO_TICKS(5));
  }
}

//电机控制任务
void controlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz控制频率（2ms间隔）
    static float prev_elec_angle = 0.0f; // 上次电气角度
    
    // 开环控制变量
    static float openloop_angle = 0;      // 开环角度累积
    static uint32_t last_time = 0;        // 上次时间
    const TickType_t controlPeriod = 2; // 2ms控制周期
    
    for (;;) {
        control_data_t local_data;  // 本地数据副本，减少锁持有时间
        
        // 获取最新的控制数据（加锁保护）
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_data = control_data;  // 复制共享数据到本地
            xSemaphoreGive(dataMutex);
        }
        
        // 只有电机为使能的情况下才进行控制
        if (local_data.enable == true){
          if(local_data.vel_enable == true){
            uint32_t current_time = micros();
            if (last_time > 0) {
                float dt = (current_time - last_time) / 1000000.0f; // 转换为秒
              
                // 计算角度增量（弧度）
                // 目标速度单位：度/秒，转换为弧度/秒
                float velocity_rad_per_sec = local_data.target_velocity * PI / 180.0f;
                
                // 积分得到机械角度（弧度）
                openloop_angle += velocity_rad_per_sec * dt;
                
                // 限制角度在0-2π范围内
                if (openloop_angle > 2 * PI) {
                    openloop_angle -= 2 * PI;
                } else if (openloop_angle < 0) {
                    openloop_angle += 2 * PI;
                }
                
                // 设置固定幅值的电压，通过角度变化实现速度控制
                float Uq = constrain(openloop_gain * fabs(local_data.target_velocity), 0.1f, 6.0f);
                
                // 计算电气角度（考虑极对数）
                float elec_angle = fmod(openloop_angle * pole_pairs, 2 * PI);
                
                // 输出开环控制电压
                OutputValtage(Uq, 0, -elec_angle);  // 注意电气角度取反，取决于电机相序
                
                // 可选：调试输出
                // Serial.printf("Uq: %.2f, Target_vel: %.2f, Angle: %.2f rad\n", 
                //               Uq, local_data.target_velocity, openloop_angle);
            }
            last_time = current_time;
          }
          else if(local_data.ang_enable == true){
            uint32_t current_time = micros();
            float dt = 0.0f;
            
            // 计算时间间隔
            if (last_time > 0) {
                dt = (current_time - last_time) / 1000000.0f; // 转换为秒
            } else {
                dt = controlPeriod / 1000.0f; // 初始化为控制周期
            }
            
            // 1. 计算位置误差
            float error = angleDifference(local_data.target_angle, local_data.current_angle);
            
            // 2. 调用PID控制器计算输出电压
            float Uq = positionPidController(error, dt);
            
            // 3. 输出电压限制
            Uq = constrain(Uq, -6.0f, 6.0f);
            
            // 4. 获取当前角度并计算电气角度
            float current_angle = readAngleSafe();
            float elec_angle = 0.0f;
            
            if (!isnan(current_angle)) {
                elec_angle = calibratedEleAngle(current_angle, pole_pairs);
                prev_elec_angle = elec_angle;
            } else {
                // 角度读取失败，使用上次值
                elec_angle = prev_elec_angle;
            }
            
            // 5. 输出电压控制电机
            OutputValtage(Uq, 0, -elec_angle);
            
            last_time = current_time;
          }
        }else {
            // 电机失能，停止输出
            OutputValtage(0, 0, 0);
            // 重置开环变量
            openloop_angle = 0;
            last_time = 0;
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
// 串口通信任务 - 处理用户输入和状态输出
void serialTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        // 处理串口输入（设置目标角度）
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');  // 读取一行输入
            input.trim();  // 去除首尾空白字符
            
            // float new_target = input.toFloat();  // 转换为浮点数
            // 验证输入有效性
            // if (!isnan(new_target) && new_target >= 0 && new_target <= 360) {
            //     // 安全更新目标角度
            //     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            //         control_data.target_angle = new_target;
            //         xSemaphoreGive(dataMutex);
            //         Serial.printf("New target: %.2f\n", new_target);  // 确认消息
            //     }
            // }

            //使能电机
            if (input.equals("ENABLE")){
              if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                control_data.enable = true;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("MOTOR ENABLED!");
            }

            //失能电机
            else if(input.equals("DISABLE")){
              if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                control_data.vel_enable = false;
                control_data.ang_enable = false;
                control_data.enable = false;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("MOTOR DISABLED!");
            }
            
            //使能位置环
            else if (input.equals("ANG_ENABLE")){
              if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                control_data.ang_enable = true;
                control_data.vel_enable = false;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("ANGLE CONTROL ENABLED!");
            }

            //失能位置环
            else if(input.equals("ANG_DISABLE")){
              if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                control_data.ang_enable = false;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("ANGLE CONTROL DISABLED!");
            }

            //使能速度环
            else if (input.equals("VEL_ENABLE")){
              if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                control_data.vel_enable = true;
                control_data.ang_enable = false;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("VELOCITY CONTROL ENABLED!");
            }

            //失能速度环
            else if(input.equals("VEL_DISABLE")){
              if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                control_data.vel_enable = false;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("VELOCITY CONTROL DISABLED!");
            }

            //动态设置Kp
            else if (input.startsWith("KP ")){
              float kp = input.substring(3).toFloat();
              if (!isnan(kp) && kp >= 0){
                velocity_pid.Kp = kp;
                Serial.printf("KP set to: %.3f\n",kp);
              }
            }

            //动态设置Ki
            else if (input.startsWith("KI ")){
              float ki = input.substring(3).toFloat();
              if (!isnan(ki) && ki >= 0){
                velocity_pid.Ki = ki;
                Serial.printf("KI set to: %.3f\n",ki);
              }
            }

            //动态设置Kd
            else if (input.startsWith("KD ")){
              float kd = input.substring(3).toFloat();
              if (!isnan(kd) && kd >= 0){
                velocity_pid.Kd = kd;
                Serial.printf("KD set to: %.3f\n",kd);
              }
            }

            //动态设置目标速度
            else if (input.startsWith("TVEL ")){
              float target_vel = input.substring(5).toFloat();
              if (!isnan(target_vel)){
                if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                  control_data.target_velocity = target_vel;
                  xSemaphoreGive(dataMutex);
                }
                Serial.printf("Target velocity set to: %.2f\n",target_vel);
              }
            }

            //动态设置目标位置
            else if (input.startsWith("ANG ")){
              float target_ang = input.substring(4).toFloat();
              if (!isnan(target_ang)){
                if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5)) == pdTRUE){
                  control_data.target_angle = target_ang;
                  xSemaphoreGive(dataMutex);
                }
                Serial.printf("Target angle set to: %.2f\n",target_ang);
              }
            }
        }
        
        // 准备输出状态信息
        control_data_t local_data;
        // 获取当前数据快照
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_data = control_data;
            xSemaphoreGive(dataMutex);
        }
        
        // 输出格式化状态信息
        Serial.printf("%.2f,%.2f,%.2f\n",
                     local_data.target_velocity, 
                     local_data.current_angle, 
                     local_data.current_velocity);
        // 10Hz输出频率（100ms间隔）
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

// 系统监控任务 - 资源使用情况报告
void monitorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        // 输出内存使用情况
        Serial.printf("Free heap: %d bytes, Min free: %d bytes\n",
                     esp_get_free_heap_size(),          // 当前空闲内存
                     esp_get_minimum_free_heap_size()); // 历史最小空闲内存
        
        // 5秒报告一次
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
    }
}

void setup(){
  Serial.begin(115200);
  Motor_init(PWMA,PWMB,PWMC,ENABLE_PIN);
  Init_AS5600();
  calibrateMotor();

  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  float initial_angle = readAngleSafe();
  if(!isnan(initial_angle)){
    control_data.current_angle = initial_angle;
    control_data.target_angle = initial_angle;
  }

  Serial.println("FreeRTOS Motor Control System Initialized");

  xTaskCreatePinnedToCore(
    velocityTask,       //任务函数
    "velocityTask",     //任务名称
    4096,               //堆栈大小
    NULL,               //任务参数
    3,                  //优先级
    &velocityTaskHandle,//任务句柄
    1                   //运行核心（1）
  );

  // 创建电机控制任务（核心1，优先级4 - 最高）
  xTaskCreatePinnedToCore(
      controlTask,      // 任务函数
      "ControlTask",    // 任务名称
      4096,            // 堆栈大小
      NULL,            // 参数
      4,               // 最高优先级，确保实时性
      &controlTaskHandle,
      1                // 核心1
  );

  // 创建串口通信任务（核心0，优先级2）
  xTaskCreatePinnedToCore(
      serialTask,       // 任务函数
      "SerialTask",     // 任务名称
      4096,            // 堆栈大小
      NULL,            // 参数
      2,               // 中等优先级
      &serialTaskHandle,
      0                // 核心0
  );
  
  // 创建系统监控任务（不限核心，优先级1 - 最低）
  xTaskCreate(
      monitorTask,      // 任务函数
      "MonitorTask",    // 任务名称
      2048,            // 较小堆栈
      NULL,            // 参数
      1,               // 最低优先级
      NULL             // 不需要句柄
  );

    // 删除setup任务本身，FreeRTOS调度器将接管控制
    vTaskDelete(NULL);
}

void loop() {
    // FreeRTOS调度器启动后，传统loop函数不再需要
    vTaskDelete(NULL);  // 删除loop任务
}

// FreeRTOS堆栈溢出钩子函数
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    Serial.printf("Stack overflow in task: %s\n", pcTaskName);
    while(1);  // 死循环，需要外部复位
}

// FreeRTOS内存分配失败钩子函数
void vApplicationMallocFailedHook(void) {
    Serial.println("Memory allocation failed!");
    while(1);  // 死循环，需要外部复位
}
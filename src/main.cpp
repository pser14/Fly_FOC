#include "AS5600_foc.h"
#include "Fly_foc.h"
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

TaskHandle_t controlTaskHandle = NULL;    // 电机控制任务句柄
TaskHandle_t serialTaskHandle = NULL;     // 串口通信任务句柄
TaskHandle_t velocityTaskHandle = NULL;   // 速度计算任务句柄
QueueHandle_t angleQueue = NULL;          // 角度队列（声明但未使用）
QueueHandle_t commandQueue = NULL;        // 命令队列（声明但未使用）
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
        float raw_velocity = (angle_diff * 100000.0f) / delta_time;
        float filtered_velocity = lowPassFillter(raw_velocity,filtered_velocity,filter_alpha);

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
    
    for (;;) {
        control_data_t local_data;  // 本地数据副本，减少锁持有时间
        
        // 获取最新的控制数据（加锁保护）
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_data = control_data;  // 复制共享数据到本地
            xSemaphoreGive(dataMutex);
        }
        
        //只有电机为使能的情况下才进行控制
        if (local_data.enable == true){
        // float error = angleDifference(local_data.target_angle, local_data.current_angle);
        // float Uq = pidController(error, 0.002);
        
        // Uq = constrain(Uq, -6.0, 6.0);

        float error = control_data.target_velocity - control_data.current_velocity;
        float Uq = velocityPidcontroller(error,0.02);
        float elec_angle = EleAngle(local_data.current_angle,pole_pairs);
        
        // float elec_angle = EleAngle(local_data.current_angle, pole_pairs);
        OutputValtage(Uq, 0,-elec_angle);  // 注意电气角度取反，取决于电机相序
        }
        
        else{
          OutputValtage(0,0,0);
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
                control_data.enable = false;
                xSemaphoreGive(dataMutex);
              }
              Serial.printf("MOTOR DISABLED!");
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
            else if (input.startsWith("KD ")){
              float ki = input.substring(3).toFloat();
              if (!isnan(ki) && ki >= 0){
                velocity_pid.Ki = ki;
                Serial.printf("KI set to: %.3f\n",ki);
              }
            }

            //动态设置Kd
            else if (input.startsWith("KI ")){
              float kd = input.substring(3).toFloat();
              if (!isnan(kd) && kd >= 0){
                velocity_pid.Kd = kd;
                Serial.printf("KD set to: %.3f\n",kd);
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
        Serial.printf("Target_vel:%.2f, Current_ang:%.2f, Current_Vel:%.2f\n",
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
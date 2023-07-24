#include <Arduino.h>
#include <Wheel_2804.h>
#include <MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define STATE_STOP 0
#define STATE_RUNNING 1
#define STATE_PICKUP 2

void task_motor_move(void *pvParameters);
void task_MPU6050_Update(void *pvParameters);

StaticTask_t task_control_task_buffer;
StackType_t task_control_stack[8192];

StaticTask_t task_MPU6050_task_buffer;
StackType_t task_MPU6050_stack[4096];

StaticTask_t task_Oled_task_buffer;
StackType_t task_Oled_task_stack[4096];


float translate_speed;//直行速度
float rotate_speed;//旋转速度

float angle_pitch=50;
float angle_last_pitch;
// float angle_pitch_offset = 11.5;
float angle_pitch_offset = 0;//初始情况下的俯仰角--偏移量
float angle_target_pitch = 0;
float angle_P = 0.0315;
float angle_D = 0.0015;


// 左右轮-速度
float speed_target = 0;
float speed_P = 1.3;
float speed_I = 0.0035;
float speed_I_sum = 0;
float speed_error = 0;
float speed_right, speed_left, speed_average;
float motor_output_right;
float motor_output_left;

uint8_t running_state = STATE_PICKUP;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire); // oled屏幕初始化
void OLED_init();
void setup()
{
    // 串口初始化
    Serial.begin(115200);
    // 各种初始化：
    MPU6050_Init(); // 陀螺仪
    Wheel_init();//电机
    OLED_init();
    

    // 启动线程
     xTaskCreateStaticPinnedToCore(
         task_motor_move, "task_Motor", 4096, NULL, 2, task_control_stack, &task_control_task_buffer, 0);
    xTaskCreateStaticPinnedToCore(
        task_MPU6050_Update, "task_MPU6050", 4096, NULL, 3, task_MPU6050_stack, &task_MPU6050_task_buffer, 1);
}

void loop()
{
}
void OLED_init()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.display();
    delay(500); // Pause for 2 seconds
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setRotation(0);

    display.clearDisplay();
    display.setCursor(1, 0);
}
void task_motor_move(void *pvParameters)
{
    while (1)
    {
        // iterative setting FOC phase voltage
        motor1.loopFOC();
        motor.loopFOC();
        // Serial.printf("d:%f,%f\n",motor_1.shaftVelocity(),motor_0.shaftVelocity());
        motor1.move(motor_output_left);
        motor.move(motor_output_right);

        // // // iterative function setting the outter loop target
        // mpu.update();

        speed_right = motor.shaftVelocity();
        speed_left = motor1.shaftVelocity();
    }
}
void task_MPU6050_Update(void *pvParameters)
{
    AngleData Angle;

    while (1)
    {
        Angle = UpdateAngle();
        // //------串口读取
        Serial.print("Angle.Pitch:");
        Serial.print(Angle.Pitch);
        Serial.print("Angle.Roll:");
        Serial.println(Angle.Roll);
        //-----显示角度
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Pitch:");
        display.println(Angle.Pitch, 2); // 2---是显示小数点后几位的意思
        display.print("Roll:");
        display.println(Angle.Roll, 2);
        //------平衡模式

        speed_average = (speed_right + speed_left) / 2;//获取平均速度

        speed_target = translate_speed;//直线速度==目标速度

        speed_error = speed_target - speed_average;//目标速度-平均速度=差值

        speed_I_sum += (speed_error * speed_I);
        if (speed_I_sum > 15)
        {
            speed_I_sum = 15;
        }
        if (speed_I_sum < -15)
        {
            speed_I_sum = -15;
        }

        angle_target_pitch = speed_P * speed_error + speed_I_sum;

        motor_output_left = angle_P * ((angle_pitch - angle_pitch_offset) - angle_target_pitch) + angle_D * Angle.GyroY ; //+ rotate_speed
        motor_output_right = angle_P * ((angle_pitch - angle_pitch_offset) - angle_target_pitch) + angle_D * Angle.GyroY;//- rotate_speed
        angle_last_pitch = angle_pitch;

        display.display();
    }
}

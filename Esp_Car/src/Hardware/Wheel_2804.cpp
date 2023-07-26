#include <Wheel_2804.h>

float target_velocity = 0;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
//电机参数
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(33, 32, 13, 16);//-------扩展板的io设置--确认完毕

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(25, 26, 27, 14);//-------扩展板的io设置--确认完毕

void Wheel_init()
{
    I2Cone.begin(19, 18, 400000UL); //---0de1
    I2Ctwo.begin(23, 5, 400000UL);//我尝试把他们丢到同一条总线上面---但是实际效果却不行-----找到了地址---并且还是会出现报错tx的缓冲区是空的
    sensor.init(&I2Cone);
    sensor1.init(&I2Ctwo);
    // 连接motor对象与传感器对象
    motor.linkSensor(&sensor);
    motor1.linkSensor(&sensor1);

    // 供电电压设置 [V]
    driver.voltage_power_supply = 12; 
    driver.init();

    driver1.voltage_power_supply = 12;
    driver1.init();
    // 连接电机和driver对象
    motor.linkDriver(&driver);
    motor1.linkDriver(&driver1);

    // FOC模型选择
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // 运动控制模式设置---下面是速度模式
    //   motor.controller = MotionControlType::velocity;// torque velocity
    //   motor1.controller = MotionControlType::velocity;

    motor.phase_resistance = 5.1;                 // 相电阻=5.1
    motor1.phase_resistance = 5.1;                // 相电阻=5.1
                                                  // 当前是力矩模式
    motor.controller = MotionControlType::torque; // torque velocity
    motor1.controller = MotionControlType::torque;

    // 速度PI环设置

    // 左轮---M1:
    motor.PID_velocity.P = 0.07;
    motor.PID_velocity.I = 5;
    motor.voltage_limit = 9;      // 最大电压限制电机
    motor.LPF_velocity.Tf = 0.01; // 速度低通滤波时间常数
    motor.velocity_limit = 100;   // 设置最大速度限制
    // 右轮----M0
    motor1.PID_velocity.P = 0.2;
    motor1.PID_velocity.I = 5;
    motor1.voltage_limit = 9;      // 最大电压限制电机
    motor1.LPF_velocity.Tf = 0.01; // 速度低通滤波时间常数
    motor1.velocity_limit = 100;   // 设置最大速度限制

    // 电流限制
    motor.current_limit = 1.5;
    motor1.current_limit = 1.5;


    // motor.useMonitoring(Serial);
    // motor1.useMonitoring(Serial);

    // 初始化电机
    motor.init();
    motor1.init();
    // 初始化 FOC
    motor.initFOC();
    motor1.initFOC();
    //这个是校准这个编码器用的
    // motor_0.initFOC(4.53, Direction::CW);
    // motor_1.initFOC(3.33, Direction::CCW);
    //取消采样
    // motor.motion_downsample=0;
    // motor1.motion_downsample=0;

    Serial.println(F("Motor ready."));
}

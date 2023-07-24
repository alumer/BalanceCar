#ifndef __WHEEL_2804_H
#define __WHEEL_2804_H

#include <SimpleFOC.h>


extern BLDCMotor motor ;
extern BLDCMotor motor1 ;

//命令设置
// 
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void Wheel_init();


#endif 
#ifndef  __MPU6050_H
#define __MPU6050_H

#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <SimpleFOC.h>

struct AngleData{
    float Pitch;
    float Roll;
    float GyroY;
};


void MPU6050_Init();
AngleData UpdateAngle();
extern void Test();

#endif 
#ifndef  __OLED_UI_H
#define __OLED_UI_H

#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
void Oled_Init();

#endif 
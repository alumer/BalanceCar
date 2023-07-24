#include <OLED_Ui.h>

void Oled_Init(){
    display.display();
    delay(500); // Pause for 2 seconds
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setRotation(0);
}

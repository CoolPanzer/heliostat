#include "config.h"

int LED_STATE = 0;

AdafruitIO_Feed *batPercent = io.feed("batPercent");

void setup(){
    Serial.begin(115200);
    pinMode(13, OUTPUT);
}

void loop(){
    io.run();
    LED_STATE = abs(LED_STATE - 1);
    digitalWrite(13, LED_STATE);
    delay(2000);
    
}
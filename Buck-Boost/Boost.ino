int LED_STATE = 0;

void setup(){
    pinMode(13, OUTPUT);
}

void loop(){
    LED_STATE = abs(LED_STATE - 1);
    digitalWrite(13, LED_STATE);
    delay(2000);
}
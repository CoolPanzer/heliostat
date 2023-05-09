#include "config.h"
#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_PWMServoDriver.h>

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  10   

#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  264 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 4;

AdafruitIO_Feed *batPercent = io.feed("batPercent");     
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup(){
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    print_wakeup_reason();
    boost();
    //wifi();
    }

void boost(){
    Serial.println("Starting pwm");
    digitalWrite(LED_BUILTIN, HIGH);//turn booster on
    //  servo.display()
    delay(2000);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(2000);
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(servonum, 0, pulselen);
        Serial.println(pulselen);
    }

    delay(1000);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(servonum, 0, pulselen);
        Serial.println(pulselen);
    }
    digitalWrite(LED_BUILTIN, LOW);//turn booster on
    delay(1000);
}

void wifi(){
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    io.connect();
    int wifiTry = 0;
    while((io.status() < AIO_CONNECTED) && wifiTry < 10){
        Serial.println(io.statusText());
        wifiTry++;
        delay(500);
    }   
    if(wifiTry >= 10){
        Serial.println("Cycle failed");
    }
    else{
        io.run();
        delay(500);
        for(int i = 0; i < 5; i++){
            Serial.println(i);
            batPercent->save(i);
            delay(2000);
        }
        Serial.println("Cycle success");
    }
    Serial.println("Going to sleep now");
    delay(500);
    Serial.flush(); 
    esp_deep_sleep_start();
}


void loop(){
}

/*
|------------------------------|
#include "config.h"

#include "Adafruit_LC709203F.h"
#include "Adafruit_AHTX0.h"

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  10        
#define DISCONNECT_TO_SLEEP  6        


Adafruit_AHTX0 aht;
Adafruit_LC709203F lc;

AdafruitIO_Feed *tmp = io.feed("temperature");
AdafruitIO_Feed *batVoltage = io.feed("batVoltage");
AdafruitIO_Feed *batPercent = io.feed("batPercent");

RTC_DATA_ATTR int bootCount = 0;

int wifiTry = 0;                    //  ammount of tries to connect to internet/starts att 

void print_wakeup_reason(){
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
}

void blink_led(int rep, int delayTime){
    for(byte i = 0; i < rep; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayTime);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delayTime);
        Serial.println("Blink done");

    }
}

void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    delay(1000);

    blink_led(2, 100);
    io.connect();
    while((io.status() < AIO_CONNECTED) && (wifiTry < 9)) {
        Serial.println(io.statusText());
        wifiTry++;
        delay(500);
    }
    if (wifiTry >= 9) {
        blink_led(3, 100);
        Serial.println("Failed to connect. Commencing deepsleep for 1 minute!");
        esp_sleep_enable_timer_wakeup(DISCONNECT_TO_SLEEP * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }

    blink_led(1, 1000);
    Serial.println();
    Serial.println(io.statusText());

    if (! aht.begin()) {
        Serial.println("Could not find AHT? Check wiring");
        while (1) delay(10);
    }
        Serial.println("AHT10 or AHT20 found");

    if (!lc.begin()) {
        Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
        while (1) delay(10);
    }
    Serial.println(F("Found LC709203F"));
    Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    lc.setThermistorB(3950);
    Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    lc.setPackSize(LC709203F_APA_1000MAH);

    lc.setAlarmVoltage(3.8);

    io.run();
    delay(500);

    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    print_wakeup_reason();

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Batt Voltage: "); Serial.println(lc.cellVoltage(), 3);
    Serial.print("Batt Percent: "); Serial.println(lc.cellPercent(), 1);
    Serial.print("Batt Temp: "); Serial.println(lc.getCellTemperature(), 1);

    tmp->save(temp.temperature);
    batPercent->save(lc.cellPercent());
    batVoltage->save(lc.cellVoltage());

    Serial.println("Going to sleep now");
    Serial.flush(); 
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}

void loop(){
}
*/
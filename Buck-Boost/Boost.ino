#include "config.h"

int LED_STATE = 0;

AdafruitIO_Feed *batPercent = io.feed("batPercent");

void setup(){
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    delay(1000);
    io.connect();
    while((io.status() < AIO_CONNECTED)){
        Serial.println(io.statusText());
        delay(500);
    }
}

void boost(){
    delay(500);
    LED_STATE = abs(LED_STATE - 1);
}

void wifi(){
    for(int i = 0; i < 5; i++){
        Serial.println(LED_STATE);
        batPercent->save(LED_STATE);
        delay(2000);
    }
}

void loop(){
    io.run();
    boost();
    wifi();
    digitalWrite(13, LED_STATE);
    delay(5000);
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
#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"
#include <Adafruit_INA219.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_LC709203F.h"
 
#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/rtc.h"
#else 
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#include "rom/rtc.h"
#endif

Adafruit_LC709203F lc;
Adafruit_INA219 ina219;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // make sure too solder together the A0 pads on the bottom of the featherwing
 
 
#define SERVOMIN  265 // This is the 'minimum' pulse length count (out of 4096) (remember 590)
#define SERVOMAX  375 // This is the 'maximum' pulse length count (out of 4096)(remember 3000)
#define SERVO_FREQ 50 //Servo frequency for digital servos                     (remember 300)
#define uS_TO_S_FACTOR 1000000ULL
#define NormalSleepTime 300     /* Time ESP32 will go to sleep (in seconds) */
#define NightSleepTime 900      //  Time ESP32 will go to sleep (in seconds) during the night)
#define dayStart 6              //  Time when the ESP32 will think it has turned from night to day
#define dayEnd 20               //  Time when the ESP32 will think it has turned from day to night
int HardwareFails;              //  Counted amount of failures by the hardware
int pulseleng;                  //  Variable distance the servo has to move depending on the time of day
int reset_reason;               //  Reason for reset, will be used to determine if brownout has occured
long long deltaTime;        //  Due to the code calculating the servos movement ahead of time (before going to sleep), when connecting to the internet the difference in time can be no longer than 5 minutes
long long errorTime;
int gotHour;                    //  Indicator of having received the time from internet
bool got_Time;
long startTime = false;                 //  From code having started record the time in miliseconds with millis()
uint8_t servoUpDown = 4;        //  The port that the servo responsible for elevation is connected to
uint8_t servoSide = 7;          //  The port that the servo responsible for azimuth is connected to
int azipulse;                   //  Distance in azimuth
int elepulse;                   //  Distance in elevation
int enable_booster = 6;         //  Pin 6 from the ESP32 that is connected to the enable path on the buckboost
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;
float Lon = 13.38 * DEG_TO_RAD,
      Lat = 55.34 * DEG_TO_RAD,
      elevation,
      azimuth;
int sun_azimuth;    //  Suns position in azimuth
int sun_elevation;  //  Suns position in elevation
//String time_str, current_hour, current_minute, current_day, current_month, current_year;
 
int nightHour;      //  Time of night hour
int nightMin;       //  Time of night minutes
 
//RTC_DATA_ATTR values
RTC_DATA_ATTR long time2sleep;  //  Time the ESP32 goes to sleep for
RTC_DATA_ATTR int WifiFailures; //  Times the ESP32 failed to connect to wifi
RTC_DATA_ATTR int inaFailures;  //  Times the ESP32 failed to connect to INA219
RTC_DATA_ATTR int fgFailures;   //  Times the ESP32 failed to connect to fuel gauge
int timeHour;   //  current time's hour
int timeMin;    //  current time's minutes
int timeDay;    //  current day
int timeMonth;  //  current month
int timeYear;   //  current year
RTC_DATA_ATTR int fTimeHour;  //  future time's hour
RTC_DATA_ATTR int fTimeMin;   //  future time's minutes
RTC_DATA_ATTR int fTimeDay;   //  future day
RTC_DATA_ATTR int fTimeMonth; //  future month
RTC_DATA_ATTR int fTimeYear;  //  future year
RTC_DATA_ATTR long long futureTime; //  future time in epoch format
long long currentTime;
RTC_DATA_ATTR int brownOut;   //  Times the ESP32 experienced brownout
float fGaugeVoltage;            //  Voltage read from fuel gauge
float fGaugePercentage;         //  Battery percent read from fuel gauge 
float INA219_current_avg = 0;   //  Average current of 1000 values read from INA
 
AdafruitIO_Group *group = io.group("gyarb4-heliostat");
AdafruitIO_Time *seconds = io.time(AIO_TIME_SECONDS);   //  MQTT subscription to AIO seconds since 1970
 
void setup(){
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, true);

  Serial.begin(115200);
  seconds->onMessage(handleSeconds);
  gotHour = timeHour;
  pinMode(enable_booster, OUTPUT);
 
  if(rtc_get_reset_reason(0) == 15){
    brownOut++;
    esp_sleep_enable_timer_wakeup(10800*uS_TO_S_FACTOR);
    Serial.println(brownOut);
    esp_deep_sleep_start();
  }

  int inaretries = 0;
  // INA219 code start
  while (! ina219.begin()) {
    inaretries++;
    if(inaretries < 9) {
      Serial.println("Failed to find INA219 chip");
      Serial.println(inaretries);
      delay(1000);
    }
    else  {
      Serial.println("to many tries going to sleep");
      switch (inaFailures) {
      case 0: time2sleep = 60; break;
      case 1: time2sleep = 180; break;
      case 2: time2sleep = 600; break;
      case 3: time2sleep = 1200; break;
      default: time2sleep = 3600; break;
      }
      Serial.println("Going to sleep now");
      inaFailures++;
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      Serial.println(time2sleep);
      esp_deep_sleep_start();
    }
  }
  Serial.println("ina connected");
 
  //Fuel Gauge code start
  int fgretries = 0;
  while (!lc.begin()) {
    fgretries++;
    if(fgretries < 9) {
      Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
      Serial.println(fgretries);
      delay(1000);
    }
    else  {
      Serial.println("to many tries going to sleep");
      switch (fgFailures) {
      case 0: time2sleep = 60; break;
      case 1: time2sleep = 180; break;
      case 2: time2sleep = 600; break;
      case 3: time2sleep = 1200; break;
      default: time2sleep = 3600; break;
      }
      Serial.println("Going to sleep now");
      fgFailures++;
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      Serial.println(time2sleep);
      esp_deep_sleep_start();
    }
  }

  Serial.println("fg connected");
  lc.setPackSize(LC709203F_APA_3000MAH);
  lc.setAlarmVoltage(3.8);
  fGaugeVoltage = lc.cellVoltage();
  fGaugePercentage = lc.cellPercent();
  //Fuel Gauge code end

  if (fGaugePercentage < 3) {
    time2sleep = 1800;
  }
  else if (fGaugePercentage < 4) {
    time2sleep = 1200;
  }
  else if (fGaugePercentage < 6) {
    time2sleep = 900;
  }
  else {
    time2sleep = NormalSleepTime;
  }
  HardwareFails = inaFailures + fgFailures;
  
  startTime = millis();
 
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
   
  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && brownOut == 0){
    Calculate_Sun_Position(fTimeHour, fTimeMin, 0, fTimeDay, fTimeMonth, fTimeYear);
    Move_Servo();
    deltaTime = futureTime;//...
    Connect_To_Internet();
    delay(1000);
    errorTime = abs(currentTime - deltaTime);//...
    Serial.print("delta: ");
    Serial.println(errorTime);
    if(errorTime > 300){//
      Serial.println("Time assessement was not correct");
      Calculate_Sun_Position(timeHour, timeMin, 0, timeDay, timeMonth, timeYear); // parameters are HH:MM:SS DD:MM:YY start from midnight and work out all 24 hour positions.
      Move_Servo();
    }
    Serial.print(deltaTime);
    Send_To_Internet();
  }

  else{
    led_blink(1,2000);
    Connect_To_Internet();
    delay(1000);
    led_blink(2,1000);
    Calculate_Sun_Position(timeHour, timeMin, 0, timeDay, timeMonth, timeYear);
    led_blink(3,2000);
    Move_Servo();
    led_blink(4,1000);
    Send_To_Internet();
    brownOut = 0;
  }

  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, false);
  led_blink(10,20);
  //go too sleep  
  esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
  Serial.println("End of code");
  esp_deep_sleep_start();
}

void led_blink(int rep, int time){
  for(int i = 0 ; i < rep ; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(time);
  }
}

void Move_Servo(){
  // Servo code begin

  digitalWrite(enable_booster, HIGH);
  delay(500);
  azipulse=map(sun_azimuth, 90, 270, 0, 180);//Map the sun position to degrees
  pulseleng=map(azipulse, 180, 0, SERVOMIN, SERVOMAX);// Map the degrees to PW
  Serial.println(azipulse);
  pwm.setPWM(servoSide, 0, pulseleng);//Move the servo to correct position
  delay(1000);
  pwm.setPWM(servoSide, 0, 4096);//servo "sleep"
  Serial.println("side moved");
  if (sun_elevation < 0) {
    sun_elevation = 0; // Point at horizon if less than horizon
  }
  sun_elevation = 145 - sun_elevation;
  sun_elevation = map(sun_elevation, 180, 0, SERVOMIN, SERVOMAX);// Map the degrees to PWM
  pwm.setPWM(servoUpDown, 0, sun_elevation);  // Move the servo to correct position
  delay(1000);
  pwm.setPWM(servoUpDown, 0, 4096);//servo "sleep"
  Serial.println("updown moved");
  // Servo code end
  delay(500);
  digitalWrite(enable_booster, LOW);
  delay(2000);
}

void Connect_To_Internet(){
  io.connect();
  int wifiretries = 0;
  while(io.status() < AIO_CONNECTED) {
   
    wifiretries++;
    if(wifiretries < 9) {
      Serial.println(io.statusText());
      delay(1000);
    }
    else  {
      Serial.println("to many tries going to sleep");
      switch (WifiFailures) {
      case 0: time2sleep = 60; break;
      case 1: time2sleep = 180; break;
      case 2: time2sleep = 600; break;
      case 3: time2sleep = 1200; break;
      default: time2sleep = 3600; break;
      }
      Serial.println("Going to sleep now");
      WifiFailures++;
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      Serial.println(time2sleep);
      esp_deep_sleep_start();
     
    }
  }
  Serial.println(io.statusText());
  while(!got_Time){
    io.run();
  }
}

void Send_To_Internet(){
    for(int i = 0; i < 1000; i++) {
      INA219_current_avg = INA219_current_avg + ina219.getCurrent_mA();   //Calculate the total sum
      //Serial.println(i);
    }
 
      INA219_current_avg = INA219_current_avg / 1000;//Calculate the current average
      Serial.println("ina average calculated");
      // INA219 code end

    while (gotHour == timeHour) {
    if (millis()-startTime > 20000) {
      group->set("Current_static", INA219_current_avg);
      group->set("Voltage_static", fGaugeVoltage);
      group->set("Percentage_static", fGaugePercentage);
      group->set("Wifi_Failures", WifiFailures);
      group->set("hardware-failures", HardwareFails);
      group->save();
      Serial.println("try to send data");
      switch (WifiFailures) {
        case 0: time2sleep = 60; break;
        case 1: time2sleep = 180; break;
        case 2: time2sleep = 600; break;
        case 3: time2sleep = 1200; break;
        default: time2sleep = 3600; break;
        }
      WifiFailures++;
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      //gettimeofday(&sleep_enter_time, NULL);
      Serial.println("Going to sleep...");
      esp_deep_sleep_start();
      break;
      }
    else {
      while(!got_Time){
        io.run();
      }
    }
  }
 
  if (timeHour < dayStart || timeHour >= dayEnd) {
 
    group->set("current-static", INA219_current_avg);
    group->set("voltage-static", fGaugeVoltage);
    group->set("percentage-static", fGaugePercentage);
    group->set("Wifi_Failures", WifiFailures);
    group->set("hardware-failures", HardwareFails);
    group->save();
    WifiFailures=0;
 
    time2sleep = NightSleepTime;
    esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
    //gettimeofday(&sleep_enter_time, NULL);
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  
  //Send data
  group->set("Current_static", INA219_current_avg);
  group->set("Voltage_static", fGaugeVoltage);
  group->set("Percentage_static", fGaugePercentage);
  group->set("Wifi_Failures", WifiFailures);
  group->set("hardware-failures", HardwareFails);
  group->save();
  Serial.println("sent data");
  WifiFailures=0;
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JD, JDx;
  int   zone = 0;  //Unused variable but retained for continuity
  JD      = JulianDate(year, month, day);
  JD_frac = (hour + minute / 60.0 + second / 3600.0) / 24.0 - 0.5;
  T          = JD - 2451545; T = (T + JD_frac) / 36525.0;
  L0         = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M          = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e          = 0.016708617 - 0.000042037 * T;
  C          = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f          = M + C;
  Obl        = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx        = JD - 2451545;
  GrHrAngle  = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle  = fmod(GrHrAngle, 360.0);
  L_true     = fmod(C + L0, 2 * PI);
  R          = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA         = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl       = asin(sin(Obl) * sin(L_true));
  HrAngle    = DEG_TO_RAD * GrHrAngle + Lon - RA;
  elevation  = asin(sin(Lat) * sin(Decl) + cos(Lat) * (cos(Decl) * cos(HrAngle)));
  azimuth    = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat)); // Azimuth measured east from north, so 0 degrees is North
  sun_azimuth   = azimuth   / DEG_TO_RAD;
  sun_elevation = elevation / DEG_TO_RAD;
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(elevation / DEG_TO_RAD, 0) + "\t\t" + String(azimuth / DEG_TO_RAD, 0));
}

long JulianDate(int year, int month, int day) {
  long JD;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JD = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD;
}

void print_wakeup_reason(){
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
}

void handleSeconds(char *data, uint16_t len) {
  time_t seconds = strtoul(data, 0, 10);
  seconds = seconds + 7200;
   time_t futureTime = strtoul(data, 0, 10);
  futureTime = seconds + time2sleep;//
  currentTime = seconds;
  tm* current_time = gmtime(&seconds);
  Serial.printf("time: %d\n", seconds);
  Serial.printf("hours minutes seconds %d %d %d %d %d %d\n", current_time->tm_year, current_time->tm_mon, current_time->tm_mday, current_time->tm_hour, current_time->tm_min, current_time->tm_sec);
  timeYear = current_time->tm_year;
  timeMonth = current_time->tm_mon;
  timeDay = current_time->tm_mday;
  timeHour = current_time->tm_hour;
  timeMin = current_time->tm_min;
  tm* future_time = gmtime(&futureTime);
  Serial.printf("time: %d\n", seconds);
  Serial.printf("hours minutes seconds %d %d %d %d %d %d\n", future_time->tm_year, future_time->tm_mon, future_time->tm_mday, future_time->tm_hour, future_time->tm_min, future_time->tm_sec);
  fTimeYear = future_time->tm_year;
  fTimeMonth = future_time->tm_mon;
  fTimeDay = future_time->tm_mday;
  fTimeHour = future_time->tm_hour;
  fTimeMin = future_time->tm_min;
  got_Time = true;
}

void loop() {
}

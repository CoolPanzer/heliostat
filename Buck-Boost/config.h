/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "CoolPants"
#define IO_KEY "aio_Zcqb47WVZWO006vTNARw5UreQtDs"

/******************************* WIFI **************************************/

#define WIFI_SSID "D410"
#define WIFI_PASS "Axel2018"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

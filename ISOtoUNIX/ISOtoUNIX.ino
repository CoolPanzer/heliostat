#include "config.h"

AdafruitIO_Time *millisH = io.time(AIO_TIME_MILLIS);

void setup() {
  Serial.begin(115200);
  io.connect();
  while (!Serial) ; // wait for serial port to connect. Needed for native USB
  millisH->onMessage(handleMillis);
}

void loop() {
  io.run();
  delay(500);
}

void handleMillis(char *data, uint16_t len) {
  Serial.printf("%s\n", data);
}

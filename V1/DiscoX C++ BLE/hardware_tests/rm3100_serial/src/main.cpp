#include <Arduino.h>
#include <Wire.h>

#include "rm3100.h"

static constexpr uint8_t RM3100_I2C_ADDR = 0x20;
static constexpr uint16_t RM3100_CYCLE_COUNT = 400;

RM3100 mag;

static void printReading(const RM3100::Reading &raw) {
  float ux = 0.0f;
  float uy = 0.0f;
  float uz = 0.0f;
  mag.toMicroTesla(raw, ux, uy, uz);

  Serial.print("raw x=");
  Serial.print(raw.x);
  Serial.print(" y=");
  Serial.print(raw.y);
  Serial.print(" z=");
  Serial.print(raw.z);
  Serial.print(" | uT x=");
  Serial.print(ux, 3);
  Serial.print(" y=");
  Serial.print(uy, 3);
  Serial.print(" z=");
  Serial.println(uz, 3);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    delay(10);
  }

  Wire.begin();
  Wire.setClock(400000);

  Serial.println();
  Serial.println("RM3100 serial test starting on nRF52840");

  if (!mag.begin(Wire, RM3100_I2C_ADDR, RM3100_CYCLE_COUNT)) {
    Serial.println("RM3100 init failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("Measurement time (ms): ");
  Serial.println(mag.measurementTime() * 1000.0f, 3);
}

void loop() {
  RM3100::Reading raw = mag.readSingle();
  printReading(raw);
  delay(250);
}

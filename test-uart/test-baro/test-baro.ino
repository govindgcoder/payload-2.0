#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// --- Configuration ---
#define STM32_SERIAL Serial2
#define SEALEVELPRESSURE_HPA (1013.25)

// Pins for STM32 Link
#define RX_PIN 16
#define TX_PIN 17

Adafruit_BME280 bme;  // I2C

// ===================================================================
// TASK 1: RECEIVE DATA FROM STM32 & PRINT TO PC (Downlink)
// ===================================================================
void receiveStmTextTask(void *pvParameters) {
  Serial.println("STM32 Receiver Task started.");
  static char buffer[256];  // Larger buffer for long log lines
  static int index = 0;

  for (;;) {
    // Read all available bytes from STM32
    while (STM32_SERIAL.available() > 0) {
      char incoming_char = STM32_SERIAL.read();

      // If newline, print the full line
      if (incoming_char == '\n') {
        buffer[index] = '\0';
        Serial.printf("[STM32] %s\n", buffer);
        index = 0;
      } else if (index < 255) {
        if (incoming_char != '\r') {
          buffer[index++] = incoming_char;
        }
      }
    }
    // Tiny delay to let other tasks run, but fast enough to catch text
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ===================================================================
// TASK 2: READ BAROMETER & SEND TO STM32 (Uplink)
// ===================================================================
void sendBaroTask(void *pvParameters) {
  Serial.println("Barometer Sender Task started (TEST MODE).");

  for (;;) {
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // Prepare Packet
    union {
      float f;
      byte b[4];
    } altData;
    altData.f = altitude;

    // Send Unconditionally
    STM32_SERIAL.write(0xBB);
    STM32_SERIAL.write(altData.b, 4);
    STM32_SERIAL.write(0x55);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  // 1. PC Serial (Debug)
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);
  Serial.println("\n--- ESP32 Hub (BME280 + STM32 Link) ---");

  // 2. STM32 Serial
  STM32_SERIAL.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // 3. I2C & BME280 Setup
  Wire.begin();  // Standard 21/22

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    // We don't halt here so we can still see STM32 logs if the sensor fails
  } else {
    Serial.println("BME280 Sensor Found.");
  }

  // 4. Create Tasks
  // Core 1 is good for Comms
  xTaskCreatePinnedToCore(receiveStmTextTask, "RX STM32", 4096, NULL, 1, NULL, 1);
  // Core 1 is also fine for I2C (Wire library isn't thread-safe across cores anyway)
  xTaskCreatePinnedToCore(sendBaroTask, "TX Baro", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}
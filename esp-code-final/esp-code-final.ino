#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <LoRa.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// Pin definitions:
//  Lora:
#define LoRa_NSS 15
#define LoRa_RST 14
#define LoRa_DIO0 2
#define LoRa_MOSI 23
#define LoRa_MISO 19
#define LoRa_SCK 18
// SD:
#define SD_CS 5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18
// ESP CAM:
#define CAM_TRIGGER 12

#define STM32_SERIAL Serial2
#define SEALEVELPRESSURE_HPA (1013.25)

// Pins for STM32 Link
#define RX_PIN 16
#define TX_PIN 17

Adafruit_BME280 bme; // I2C

// data structure definitions
struct LogMessage {
  char text[128];
};

QueueHandle_t dataQueue;

// function prototypes
void sentBaroTask(void *pvParameters);
void receiveBaroTask(void *pvParameters);
void storageTask(void *pvParameters);
void receiveStmTextTask(void *pvParameters);

// Initialization of systems
void setup() {
  Serial.begin(115200);
  // Wait a moment for power to stabilize
  delay(1000); 
  Serial.println("Serial Port Connected!");

  // --- 1. PIN SAFETY (Deselect Everything) ---
  pinMode(LoRa_NSS, OUTPUT);
  pinMode(LoRa_RST, OUTPUT);
  pinMode(LoRa_DIO0, INPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(CAM_TRIGGER, OUTPUT);

  // CRITICAL: Pull both CS pins HIGH to release the SPI bus
  digitalWrite(LoRa_NSS, HIGH);
  digitalWrite(SD_CS, HIGH);
  
  // Camera trigger default state
  digitalWrite(CAM_TRIGGER, LOW);

  // --- 2. INITIALIZE SD CARD (FIRST) ---
  // We use 4MHz to be safe with wiring
  Serial.print("Initializing SD... ");
  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("FAILED!");
    // We don't return, so we can at least try LoRa
  } else {
    Serial.println("OK!");
    
    // Test Write
    File dataFile = SD.open("/data.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println("--- New Session ---");
      dataFile.close();
      Serial.println("File Write Test OK.");
    } else {
      Serial.println("File Open FAILED (Check Partition Scheme/FAT32).");
    }
  }

  // --- 3. INITIALIZE LORA (SECOND) ---
  Serial.print("Initializing LoRa... ");
  LoRa.setPins(LoRa_NSS, LoRa_RST, LoRa_DIO0);
  
  if (!LoRa.begin(433E6)) {
    Serial.println("FAILED!");
    // return; // Optional: Comment this out if you want to run without Radio
  } else {
    Serial.println("OK!");
  }

  // --- 4. SENSORS & STM32 ---
  STM32_SERIAL.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Wire.begin();
  if (!bme.begin(0x76)) {
    Serial.println("BME280 Not Found!");
  }

  // --- 5. TASKS ---
  dataQueue = xQueueCreate(20, sizeof(LogMessage));

  xTaskCreatePinnedToCore(receiveStmTextTask, "RX_STM", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sentBaroTask, "TX_BARO", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(storageTask, "STORAGE", 8192, NULL, 1, NULL, 0);
}

void loop() {
	//kill arduino loop task to save RAM
  vTaskDelete(NULL);
}

void sentBaroTask(void *pvParameters) {
  Serial.println("Starting sentBaroTask");
  for (;;) {
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
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

void receiveStmTextTask(void *pvParameters) {
  static char buffer[128];
  static int idx = 0;
  static int decimator = 0; // Counts packets to throttle speed
  LogMessage msg;

  for (;;) {
    // Process all available bytes to keep UART buffer empty
    while (STM32_SERIAL.available()) {
      char c = STM32_SERIAL.read();

      if (c == '\n') {
        buffer[idx] = '\0'; // Null-terminate string

        // --- DECIMATION LOGIC ---
        // STM32 sends at 100Hz. We want 10Hz.
        // So we only process every 10th packet.
        decimator++;
        if (decimator >= 10) {
          decimator = 0; // Reset counter

          // Prepare the message
          strncpy(msg.text, buffer, sizeof(msg.text) - 1);
          msg.text[sizeof(msg.text) - 1] = '\0';

          // Send to the heavy lifting task (Core 0)
          // If queue is full, we drop it (timeout 0) to avoid blocking UART
          xQueueSend(dataQueue, &msg, 0);
        }

        // Reset buffer index for the next line (whether we sent it or not)
        idx = 0;

      } else if (idx < 127 && c != '\r') {
        buffer[idx++] = c;
      }
    }
    // Tiny yield to keep the Watchdog happy
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void storageTask(void *pvParameters) {
  LogMessage rxMsg;
  String finalLog;

  // Camera Timing Variables
  unsigned long lastCamTrigger = 0;
  const long CAM_INTERVAL = 20000; // 20 Seconds

  for (;;) {
    // 1. Wait for data from Queue (Timeout 10ms to check Camera logic even if
    // no data)
    if (xQueueReceive(dataQueue, &rxMsg, 10 / portTICK_PERIOD_MS) == pdTRUE) {

      // 2. Read Local Sensors (BME280)
      float temp = bme.readTemperature();
      float hum = bme.readHumidity();

      // 3. Format Final String: "STM_Data,Temp,Hum"
      finalLog =
          String(rxMsg.text) + "," + String(temp, 2) + "," + String(hum, 1);

      // 4. Log to SD
      File f = SD.open("/data.txt", FILE_APPEND);
      if (f) {
        f.println(finalLog);
        f.close();
      }

      // 5. Transmit LoRa
      // Note: This blocks for ~50-100ms.
      // During this time, receiveStmTextTask fills the queue.
      LoRa.beginPacket();
      LoRa.print(finalLog);
      LoRa.endPacket();

      // Debug
      Serial.println(finalLog);
    }

    // 6. Camera Trigger Logic (Non-blocking check)
    if (millis() - lastCamTrigger >= CAM_INTERVAL) {
      lastCamTrigger = millis();
      Serial.println("Triggering Camera...");
      digitalWrite(CAM_TRIGGER, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(CAM_TRIGGER, LOW);
    }
  }
}

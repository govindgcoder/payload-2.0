#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <LoRa.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
// LoRa (SX1278)
#define LoRa_NSS 15
#define LoRa_RST 14
#define LoRa_DIO0 2

// SD Card
#define SD_CS 5

// STM32 Link
#define RX_PIN 16
#define TX_PIN 17
#define STM32_SERIAL Serial2

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; 

// --- GLOBAL FLAGS ---
bool sd_ok = false;
bool lora_ok = false;

// --- DATA STRUCTURES ---
struct LogMessage {
  char text[128];
};

QueueHandle_t dataQueue;

// --- PROTOTYPES ---
void receiveStmTextTask(void *pvParameters);
void sentBaroTask(void *pvParameters);
void storageTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(1000); // Power stability
  Serial.println("\n--- MISSION HUB START ---");

  // 1. PIN SAFETY (Arbitration)
  // Deselect both SPI devices immediately to prevent bus contention
  pinMode(LoRa_NSS, OUTPUT);
  pinMode(LoRa_RST, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  
  digitalWrite(LoRa_NSS, HIGH); // Disable LoRa
  digitalWrite(SD_CS, HIGH);    // Disable SD
  
  delay(50); // Short pause to let pins settle

  // 2. INITIALIZE SD CARD (First)
  // We use 4MHz to be safe with wiring noise
  Serial.print("Initializing SD... ");
  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("FAILED!");
    sd_ok = false;
  } else {
    Serial.println("OK.");
    sd_ok = true;
    
    // Write Header
    File f = SD.open("/data.txt", FILE_APPEND);
    if (f) {
      f.println("--- NEW FLIGHT LOG ---");
      f.close();
    } else {
      Serial.println("SD Initialized but File Open Failed!");
      sd_ok = false; // Disable SD logic if we can't write
    }
  }

  // 3. INITIALIZE LORA (Second)
  Serial.print("Initializing LoRa... ");
  
  // Re-assert SD CS High just to be safe before touching LoRa
  digitalWrite(SD_CS, HIGH);
  
  LoRa.setPins(LoRa_NSS, LoRa_RST, LoRa_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("FAILED!");
    lora_ok = false;
  } else {
    Serial.println("OK.");
    lora_ok = true;
  }

  // 4. PERIPHERALS
  STM32_SERIAL.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Wire.begin();
  if (!bme.begin(0x76)) {
    Serial.println("BME280 Failed!");
  }

  // 5. TASKS
  dataQueue = xQueueCreate(20, sizeof(LogMessage));

  // Core 1 (Fast): UART RX from STM32 and Baro TX to STM32
  xTaskCreatePinnedToCore(receiveStmTextTask, "RX_STM", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sentBaroTask, "TX_BARO", 2048, NULL, 1, NULL, 1);

  // Core 0 (Slow): Storage (SD) & Telemetry (LoRa)
  xTaskCreatePinnedToCore(storageTask, "STORAGE", 8192, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL); // Free RAM
}

// --- TASKS ---

void receiveStmTextTask(void *pvParameters) {
  static char buffer[128];
  static int idx = 0;
  static int decimator = 0;
  LogMessage msg;

  for (;;) {
    while (STM32_SERIAL.available()) {
      char c = STM32_SERIAL.read();
      if (c == '\n') {
        buffer[idx] = '\0';
        // Decimate 100Hz -> 10Hz
        decimator++;
        if (decimator >= 10) {
          decimator = 0;
          strncpy(msg.text, buffer, sizeof(msg.text) - 1);
          msg.text[sizeof(msg.text) - 1] = '\0';
          xQueueSend(dataQueue, &msg, 0);
        }
        idx = 0;
      } else if (idx < 127 && c != '\r') {
        buffer[idx++] = c;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void storageTask(void *pvParameters) {
  LogMessage rxMsg;
  String finalLog;

  for (;;) {
    // Wait for data from Queue
    if (xQueueReceive(dataQueue, &rxMsg, portMAX_DELAY) == pdTRUE) {
      
      float temp = bme.readTemperature();
      float hum = bme.readHumidity();
      
      // Format: STM_Data,Temp,Hum
      finalLog = String(rxMsg.text) + "," + String(temp, 1) + "," + String(hum, 0);
      
      // 1. ALWAYS Print to Serial (PC) for debugging
      Serial.println(finalLog);

      // 2. SD Write (Only if initialized successfully)
      if (sd_ok) {
        File f = SD.open("/data.txt", FILE_APPEND);
        if (f) { 
          f.println(finalLog); 
          f.close(); 
        } else {
          // If write fails once, print error but try again next time
          Serial.println("Error: SD Write Failed");
        }
      }

      // 3. LoRa Write (Only if initialized successfully)
      if (lora_ok) {
        if (LoRa.beginPacket()) {
          LoRa.print(finalLog); 
          LoRa.endPacket();
        } else {
          Serial.println("Error: LoRa Busy/Failed");
        }
      }
    }
  }
}

void sentBaroTask(void *pvParameters) {
  for (;;) {
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    union { float f; byte b[4]; } altData;
    altData.f = altitude;

    STM32_SERIAL.write(0xBB);
    STM32_SERIAL.write(altData.b, 4);
    STM32_SERIAL.write(0x55);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
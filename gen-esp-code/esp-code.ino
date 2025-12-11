/*
  ESP32 Dual-Core FreeRTOS Data Logger & LoRa Transmitter

  - CORE 0: Handles high-frequency sensor reading and data logging to SD card.
  - CORE 1: Handles lower-frequency LoRa telemetry transmission.

  - RTOS CONCEPTS USED:
    - Tasks pinned to specific cores.
    - Semaphore (Binary): Signals the logger task on Core 0 when new data is ready.
    - Mutex (spiBusMutex): Protects the shared SPI bus for SD and LoRa.
    - Mutex (dataMutex): Protects the shared SensorData struct from concurrent access by both cores.
*/

// --- Core & Sensor Libraries ---
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <LoRa.h>  // For the LoRa module

// SPI Bus Pins (shared by SD and LoRa)
const int SD_LORA_CS_PIN = 5;
const int SPI_MOSI_PIN = 23;
const int SPI_MISO_PIN = 19;
const int SPI_SCK_PIN = 18;
// LoRa Specific Pins
const int LORA_RST_PIN = 14;
const int LORA_DIO0_PIN = 2;  // IRQ Pin
// UART for STM32 connection (using Serial2: RX=16, TX=17)
#define STM32_SERIAL Serial2
// LoRa Frequency
#define LORA_FREQUENCY 433E6

// --- Sensor Objects & Data Structure ---
Adafruit_BME280 bme;
// New struct for data from STM32
struct StmData {
  float qW = 1.0, qX = 0.0, qY = 0.0, qZ = 0.0;  // Default to a valid quaternion
  float latitude = 0.0, longitude = 0.0, altitude = 0.0;
};

struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  StmData stm_data;  // Embed the STM32 data
};
SensorData currentReadings;

// --- FreeRTOS Handles ---
TaskHandle_t readSensorsTaskHandle, logDataTaskHandle, loraTransmitTaskHandle, receiveStmDataTaskHandle;
SemaphoreHandle_t dataReadySemaphore;  // Signal for new data
SemaphoreHandle_t spiBusMutex;         // Protects SPI bus for SD/LoRa
SemaphoreHandle_t dataMutex;           // Protects the currentReadings struct

// ===================================================================
// CORE 0, TASK 1: READ SENSORS
// ===================================================================
void readSensorsTask(void *pvParameters) {
  Serial.println("Sensor Reading Task started on Core 0.");
  for (;;) {
    SensorData tempData;  // Read into a temporary local struct
    tempData.temperature = bme.readTemperature();
    tempData.humidity = bme.readHumidity();
    tempData.pressure = bme.readPressure() / 100.0F;

    // Safely update the global data structure
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      currentReadings = tempData;
      xSemaphoreGive(dataMutex);
    }

    Serial.printf("Core 0: Read Temp=%.2f, Hum=%.2f",
                  tempData.temperature, tempData.humidity);

    xSemaphoreGive(dataReadySemaphore);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// ===================================================================
// CORE 0, TASK 2: LOG DATA TO SD CARD
// ===================================================================
void logDataTask(void *pvParameters) {
  Serial.println("Data Logging Task started on Core 0.");
  for (;;) {
    if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE) {
      SensorData localData;
      // Safely get a copy of the data
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        localData = currentReadings;
        xSemaphoreGive(dataMutex);
      }

      // Safely access the SD card via SPI bus
      if (xSemaphoreTake(spiBusMutex, portMAX_DELAY) == pdTRUE) {
        File dataFile = SD.open("/datalog.csv", FILE_APPEND);
        if (dataFile) {
          dataFile.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f\n", millis(),
                          localData.stm_data.qW, localData.stm_data.qX, localData.stm_data.qY, localData.stm_data.qZ,
                          localData.stm_data.latitude, localData.stm_data.longitude, localData.stm_data.altitude,
                          localData.temperature, localData.humidity,
                          localData.pressure);
          dataFile.close();
          Serial.println("Core 0: Logged data to SD card.");
        } else {
          Serial.println("Core 0: Error opening datalog.csv");
        }
        xSemaphoreGive(spiBusMutex);
      }
    }
  }
}

// ===================================================================
// CORE 1, TASK 3: TRANSMIT LORA DATA
// ===================================================================
void loraTransmitTask(void *pvParameters) {
  Serial.println("LoRa Transmit Task started on Core 1.");
  for (;;) {
    // This task will run every 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    SensorData localData;
    // Safely get a copy of the global data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      localData = currentReadings;
      xSemaphoreGive(dataMutex);
    }

    // Create the CSV data string to send with STM32 data
    char csvString[120];
    snprintf(csvString, sizeof(csvString), "%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.2f",
             localData.stm_data.qW, localData.stm_data.qX, localData.stm_data.qY, localData.stm_data.qZ,
             localData.stm_data.latitude, localData.stm_data.longitude, localData.stm_data.altitude);
    

    // Safely access the LoRa module via SPI bus
    if (xSemaphoreTake(spiBusMutex, portMAX_DELAY) == pdTRUE) {
      LoRa.beginPacket();
      LoRa.print(csvString);
      LoRa.endPacket();
      xSemaphoreGive(spiBusMutex);
      Serial.printf("Core 1: LoRa packet sent: [%s]\n", csvString);
    }
  }
}

// --- Helper function to calculate checksum ---
byte calculateChecksum(const byte *data, size_t length) {
  byte checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum += data[i];
  }
  return checksum;
}

// ===================================================================
// CORE 1, TASK 4: RECEIVE DATA FROM STM32
// ===================================================================
void receiveStmDataTask(void *pvParameters) {
  Serial.println("STM32 UART Receiver Task started on Core 1.");
  static byte buffer[31];
  static int buffer_index = 0;
  StmData tempData;

  for (;;) {
    while (STM32_SERIAL.available() > 0) {
      byte incoming_byte = STM32_SERIAL.read();

      if (buffer_index == 0) {  // Waiting for start byte
        if (incoming_byte == 0xAA) {
          buffer[buffer_index++] = incoming_byte;
        }
      } else {  // Already started, fill the buffer
        buffer[buffer_index++] = incoming_byte;

        if (buffer_index >= 31) {    // Buffer is full
          if (buffer[30] == 0x55) {  // Check end byte
            byte calculated_checksum = calculateChecksum(&buffer[1], 28);
            if (buffer[29] == calculated_checksum) {
              // Packet is valid! Unpack it.
              memcpy(&tempData.qW, &buffer[1], 4);
              memcpy(&tempData.qX, &buffer[5], 4);
              memcpy(&tempData.qY, &buffer[9], 4);
              memcpy(&tempData.qZ, &buffer[13], 4);
              memcpy(&tempData.latitude, &buffer[17], 4);
              memcpy(&tempData.longitude, &buffer[21], 4);
              memcpy(&tempData.altitude, &buffer[25], 4);

              // Safely update the global data structure
              if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                currentReadings.stm_data = tempData;
                xSemaphoreGive(dataMutex);
              }
            } else {
              // Serial.println("Core 1: Checksum mismatch!");
            }
          }
          buffer_index = 0;  // Reset for next packet
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to other tasks
  }
}

// ===================================================================
// SETUP FUNCTION
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- ESP32 Dual-Core Logger & Transmitter ---");

  // Initialize I2C
  Wire.begin();
  if (!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1)
      ;
  }
  Serial.println("BME280 Sensor Found.");

  // Start Serial for STM32 communication
  STM32_SERIAL.begin(115200);

  // Create Semaphores and Mutexes
  dataReadySemaphore = xSemaphoreCreateBinary();
  spiBusMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  // Initialize SPI bus
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_LORA_CS_PIN);

  // Initialize SD Card (protected by SPI mutex)
  if (xSemaphoreTake(spiBusMutex, portMAX_DELAY) == pdTRUE) {
    if (!SD.begin(SD_LORA_CS_PIN)) {
      Serial.println("SD Card initialization failed!");
    } else {
      Serial.println("SD Card Initialized.");
      File dataFile = SD.open("/datalog.csv");
      if (!dataFile) {
        dataFile = SD.open("/datalog.csv", FILE_WRITE);
        dataFile.println("Timestamp,qW,qX,qY,qZ,Lat,Lon,Alt,Temperature,Humidity,Pressure,UV_Index");
      }
      dataFile.close();
    }
    xSemaphoreGive(spiBusMutex);
  }

  // Initialize LoRa Module (protected by SPI mutex)
  if (xSemaphoreTake(spiBusMutex, portMAX_DELAY) == pdTRUE) {
    LoRa.setPins(SD_LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQUENCY)) {
      Serial.println("Starting LoRa failed!");
      while (1)
        ;
    }
    Serial.printf("LoRa Initialized at %.0f MHz\n", LORA_FREQUENCY / 1E6);
    xSemaphoreGive(spiBusMutex);
  }

  // --- Create Tasks ---
  // Core 0 Tasks
  xTaskCreatePinnedToCore(readSensorsTask, "Read Sensors", 4096, NULL, 2, &readSensorsTaskHandle, 0);
  xTaskCreatePinnedToCore(logDataTask, "Log Data", 4096, NULL, 1, &logDataTaskHandle, 0);
  // Core 1 Tasks
  xTaskCreatePinnedToCore(loraTransmitTask, "Transmit LoRa", 4096, NULL, 1, &loraTransmitTaskHandle, 1);
  xTaskCreatePinnedToCore(receiveStmDataTask, "Receive STM32", 4096, NULL, 2, &receiveStmDataTaskHandle, 1);
}

void loop() {
  vTaskDelete(NULL);  // Delete the loop task
}

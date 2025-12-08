#include <Arduino.h>

// --- Configuration ---
// UART for STM32 connection (RX=16, TX=17)
#define STM32_SERIAL Serial2

// ===================================================================
// TASK: RECEIVE TEXT FROM STM32 & PRINT TO SERIAL
// ===================================================================
void receiveStmTextTask(void *pvParameters) {
  Serial.println("STM32 Text Receiver Task started on Core 1.");
  
  // Buffer to hold one line of text
  static char buffer[128];
  static int index = 0;

  for (;;) {
    while (STM32_SERIAL.available() > 0) {
      char incoming_char = STM32_SERIAL.read();

      // Check for end of line (Newline character)
      if (incoming_char == '\n') {
        buffer[index] = '\0'; // Null-terminate the string
        
        // Print the complete line to the PC Serial Monitor
        Serial.printf("[STM32] %s\n", buffer);
        
        // Reset buffer for the next line
        index = 0;
      } 
      // Add character to buffer if we have space
      else if (index < 127) {
        // Ignore carriage return '\r' to keep output clean
        if (incoming_char != '\r') {
          buffer[index++] = incoming_char;
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to other tasks
  }
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  // Debug Serial (USB connection to Computer)
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- ESP32 Listening for STM32 Printf ---");

  // UART Serial (Connection to STM32)
  // Ensure pins 16 (RX) and 17 (TX) are connected to STM32 TX and RX
  STM32_SERIAL.begin(115200, SERIAL_8N1, 16, 17);

  // Create Receiver Task on Core 1
  xTaskCreatePinnedToCore(
    receiveStmTextTask,   // Function
    "Receive STM32",      // Name
    4096,                 // Stack size
    NULL,                 // Parameters
    1,                    // Priority
    NULL,                 // Handle
    1                     // Core
  );
}

void loop() {
  vTaskDelete(NULL); // Loop not used
}
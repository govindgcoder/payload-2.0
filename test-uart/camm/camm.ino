#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "EEPROM.h"

// --- AI THINKER PIN MAP ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// The large Flash LED is on GPIO 4
#define FLASH_LED_PIN     4

int fileCounter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- CAM START ---");

  // --- 1. VISUAL FEEDBACK (FLASH TWICE) ---
  // This confirms power is good and code is running
  pinMode(FLASH_LED_PIN, OUTPUT);
  
  // Blink 1
  digitalWrite(FLASH_LED_PIN, HIGH); delay(200);
  digitalWrite(FLASH_LED_PIN, LOW);  delay(200);
  // Blink 2
  digitalWrite(FLASH_LED_PIN, HIGH); delay(200);
  digitalWrite(FLASH_LED_PIN, LOW);  delay(200);

  // --- 2. CAMERA CONFIG (SAFE MODE) ---
  // Give sensor time to wake up after the high-current flash
  delay(1000);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  // 10MHz XCLK for stability (Fixes 0x106 error)
  config.xclk_freq_hz = 10000000; 
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;  // 640x480
  config.jpeg_quality = 12;           
  config.fb_count = 2;                

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Cam Init Failed!");
    // Rapid blink to signal error
    for(int i=0; i<5; i++) {
        digitalWrite(FLASH_LED_PIN, HIGH); delay(50);
        digitalWrite(FLASH_LED_PIN, LOW); delay(50);
    }
    return;
  }

  // --- 3. SD CARD INIT ---
  if (!SD_MMC.begin("/sdcard", true)) { // 'true' = 1-bit mode
    Serial.println("SD Mount Failed");
    // Rapid blink to signal error
    for(int i=0; i<5; i++) {
        digitalWrite(FLASH_LED_PIN, HIGH); delay(50);
        digitalWrite(FLASH_LED_PIN, LOW); delay(50);
    }
    return;
  }
  
  SD_MMC.mkdir("/rec");
  Serial.println("Recording Started.");
}

void loop() {
  // 1. Capture
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture Failed");
    return;
  }

  // 2. Save
  String path = "/rec/img_" + String(fileCounter) + ".jpg";
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  
  if (file) {
    file.write(fb->buf, fb->len);
    file.close();
    if(fileCounter % 10 == 0) Serial.printf("Saved: %s\n", path.c_str());
    fileCounter++;
  }

  // 3. Cleanup
  esp_camera_fb_return(fb);

  // 4. Rate Control (Approx 10Hz)
  // Processing time + Write time + 50ms Delay ~= 100ms total
  delay(50); 
}
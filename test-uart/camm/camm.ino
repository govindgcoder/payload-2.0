#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"  // Use SD_MMC for the 4-bit connection on ESP32-CAM

// Define the camera model
#define CAMERA_MODEL_AI_THINKER

// Pin definition for the trigger input
#define TRIGGER_PIN 12

// --- Camera Pin Definition ---
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// --- Global Variables ---
// This flag is set to true by the interrupt service routine
volatile bool startRecording = false;

// Counter for naming recording folders
int recordingCounter = 0;

// This function is called when the trigger pin goes from LOW to HIGH
void IRAM_ATTR triggersRecording() {
  startRecording = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Recorder Initializing...");

  // Configure the trigger pin as an input with a pull-down resistor.
  // This ensures the pin is LOW until the other ESP32 pulls it HIGH.
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);

  // Attach an interrupt to the trigger pin.
  // It will call 'triggersRecording' on the RISING edge (LOW to HIGH transition).
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), triggersRecording, RISING);

  // --- Initialize the SD Card ---
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed!");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }
  Serial.println("SD Card initialized.");

  // --- Initialize the Camera ---
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // Important for saving images

  // For video, a lower resolution is better for speed
  config.frame_size = FRAMESIZE_VGA;  // 640x480
  config.jpeg_quality = 12;           // 0-63, lower number means higher quality
  config.fb_count = 2;                // Use 2 frame buffers for smoother capture

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  Serial.println("Initialization complete. Waiting for trigger...");
}

// This function saves a sequence of JPEGs to the SD card
void recordVideo(int frameCount) {
  recordingCounter++;
  String folderName = "/recording_" + String(recordingCounter);

  Serial.printf("Creating folder: %s\n", folderName.c_str());
  SD_MMC.mkdir(folderName);

  Serial.printf("Recording %d frames...\n", frameCount);

  for (int i = 0; i < frameCount; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;  // Skip this frame
    }

    String path = folderName + "/image_" + String(i) + ".jpg";
    fs::FS &fs = SD_MMC;
    File file = fs.open(path.c_str(), FILE_WRITE);
    if (!file) {
      Serial.printf("Failed to open file for writing: %s\n", path.c_str());
    } else {
      file.write(fb->buf, fb->len);  // Write the buffer to the file
      Serial.printf("Saved file: %s\n", path.c_str());
    }
    file.close();

    esp_camera_fb_return(fb);  // IMPORTANT: Return the frame buffer to free up memory
  }

  Serial.println("Recording finished.");
}


void loop() {
  // Check if the interrupt has set our flag
  if (startRecording) {
    Serial.println("Trigger received! Starting recording for 150 frames.");

    // Call the function to save the frames
    // 150 frames at ~15fps = ~10 seconds of video
    recordVideo(150);

    // Reset the flag so we can be triggered again
    startRecording = false;
  }
}
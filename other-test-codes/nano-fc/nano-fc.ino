#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// starting vals
const unsigned long dt_ms = 20;  // for the 50Hz Loop
const float alpha = 0.98f;       // ts for complimentary filter

const float threshold = 25.0f;             // trigger for launch detection
const float APOGEE_DROP = 0.5f;            // trigger for apogee detection
const unsigned long failsafeTime = 10000;  // Failsafe: 10 seconds

// state machine variables
enum State { IDLE,
             ASCENT,
             TRIGGERED };
             
// Start in IDLE so we can calibrate bias before launch
State flightState = IDLE; 

float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float gravityMagnitude = 9.81f;
float lin_acc_bias = 0; // Tracks bias while sitting on pad

// vector towards gravity
float grav_x = 0;
float grav_y = 0;
float grav_z = 1.0f;

float verticalVelocity = 0;
float altitude = 0;
float maxAltitude = 0;

unsigned long previousTime = 0;
unsigned long launchStartTime = 0;

void setup(void) {
  Serial.begin(115200);
  
  // SAFETY DELAY: Prevents "stk500_getsync" errors by giving the 
  // bootloader time to listen before we spam Serial prints.
  delay(2000); 

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("Calibrating...");

  sensors_event_t a, g, temp;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;

  // for basic calibration
  for (int i = 0; i < 1000; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum_gx += g.gyro.x;
    sum_gy += g.gyro.y;
    sum_gz += g.gyro.z;
    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z;
    delay(2);
  }

  // offset calc
  gx_offset = sum_gx / 1000.0f;
  gy_offset = sum_gy / 1000.0f;
  gz_offset = sum_gz / 1000.0f;

  float avg_ax = sum_ax / 1000.0f;
  float avg_ay = sum_ay / 1000.0f;
  float avg_az = sum_az / 1000.0f;
  gravityMagnitude = sqrt(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az);

  // 3. normalizing to get the initial gravity vector
  if (gravityMagnitude > 0) {
    grav_x = avg_ax / gravityMagnitude;
    grav_y = avg_ay / gravityMagnitude;
    grav_z = avg_az / gravityMagnitude;
  }

  Serial.println("Calibration done");
  Serial.print("Gravity: ");
  Serial.println(gravityMagnitude);
  
  previousTime = millis();

  //tatkalam:
  flightState = ASCENT;
}

void triggerParachute() {
  if (flightState != TRIGGERED) {
      Serial.println("PARACHUTE TRIGGERED!");
      flightState = TRIGGERED;
      // Add servo/pyro code here
  }
}

void applyComplimentaryFilter(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  // integration for gravity vector
  float nx = grav_x + (grav_y * gz - grav_z * gy) * dt;
  float ny = grav_y + (grav_z * gx - grav_x * gz) * dt;
  float nz = grav_z + (grav_x * gy - grav_y * gx) * dt;

  // Normalize to keep it a unit vector
  float norm = sqrt(nx * nx + ny * ny + nz * nz);
  if (norm > 0) {
    grav_x = nx / norm;
    grav_y = ny / norm;
    grav_z = nz / norm;
  }

  float accel_norm = sqrt(ax * ax + ay * ay + az * az);
  if (accel_norm > 0) {
    float ax_n = ax / accel_norm;
    float ay_n = ay / accel_norm;
    float az_n = az / accel_norm;

    grav_x = grav_x * alpha + ax_n * (1.0f - alpha);
    grav_y = grav_y * alpha + ay_n * (1.0f - alpha);
    grav_z = grav_z * alpha + az_n * (1.0f - alpha);

    // normalize again
    norm = sqrt(grav_x * grav_x + grav_y * grav_y + grav_z * grav_z);
    if (norm > 0) {
      grav_x /= norm;
      grav_y /= norm;
      grav_z /= norm;
    }
  }
}

void loop() {
  unsigned long currentTime = millis();

  if ((currentTime - previousTime) >= dt_ms) {
    float dt_sec = (currentTime - previousTime) / 1000.0f;
    previousTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // fix the reading with calibrated sensors
    float gx = g.gyro.x - gx_offset;
    float gy = g.gyro.y - gy_offset;
    float gz = g.gyro.z - gz_offset;

    applyComplimentaryFilter(a.acceleration.x, a.acceleration.y, a.acceleration.z, gx, gy, gz, dt_sec);

    float projection = (a.acceleration.x * grav_x) + (a.acceleration.y * grav_y) + (a.acceleration.z * grav_z);

    // subtract the gravity 'g' to get upwards motion
    float accel_linear = (projection - gravityMagnitude);

    if (flightState == IDLE) {
      // 1. Continuous Calibration:
      // While waiting, any 'linear' acceleration we see is actually error (drift/tilt).
      // We slowly learn this bias to "tare" the scale perfectly before launch.
      lin_acc_bias = lin_acc_bias * 0.95f + accel_linear * 0.05f;

      // 2. Clamp Velocity:
      // Hard reset velocity/altitude to ensure 0 drift before launch.
      verticalVelocity = 0;
      altitude = 0;

      // Check for launch (corrected for current bias)
      float adjusted_accel = accel_linear - lin_acc_bias;
      
      // Use total acceleration magnitude for launch detect (more reliable)
      float totalAccel = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
      
      if (totalAccel > threshold) {
        flightState = ASCENT;
        launchStartTime = millis();
        Serial.println("LAUNCH DETECTED!");
      }

    } else if (flightState == ASCENT) {

      // Remove the bias we learned during IDLE
      accel_linear -= lin_acc_bias;

      // Noise suppression (Deadband)
      if (abs(accel_linear) < 0.25f) {
        accel_linear = 0;
        
        // CRITICAL FIX: Velocity Decay
        // If acceleration is 0, friction/drag slows the rocket. 
        // Without this, velocity stays "stuck" at the last known speed.
        verticalVelocity *= 0.96f; 
        if (abs(verticalVelocity) < 0.05f) verticalVelocity = 0;
      }

      verticalVelocity += accel_linear * dt_sec;
      altitude += verticalVelocity * dt_sec;

      if (altitude > maxAltitude) maxAltitude = altitude;

      Serial.print("Alt:");
      Serial.print(altitude);
      Serial.print(",MaxAlt:");
      Serial.print(maxAltitude);
      Serial.print(",Vel:");
      Serial.println(verticalVelocity);

      // apogee detection
      if (altitude < (maxAltitude - APOGEE_DROP)) {
        Serial.println("apogee");
        triggerParachute();
      }

      // timer failsafe
      // if (millis() - launchStartTime > failsafeTime) {
      //   Serial.println("timer");
      //   triggerParachute();
      // }
    }
  }
}
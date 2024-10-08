#include <Wire.h>
#include <ArduinoJson.h>
#include <math.h>

const int MPU_addr1 = 0x68;
float ax, ay, az, gx, gy, gz;
float roll, pitch, yaw = 0;
float azimuth = 0;
float altitude = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
unsigned long lastUpdateTime, last_ms, OutputInterval = 100;
const float alpha = 0.98; // Complementary filter coefficient
const float beta = 0.1;   // Madgwick filter gain

// Quaternion representation of orientation
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

//Button pin setting
const int buttonPin1 = 4;
const int buttonPin2 = 5;
bool button1Pressed = false;
bool button2Pressed = false;
bool lastButtonState1 = HIGH;  // Last known state of button 1
bool lastButtonState2 = HIGH;  // Last known state of button 2

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Configure gyroscope range to ±250 deg/s for better resolution
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Serial.begin(115200);
  calibrateGyro();
  lastUpdateTime = micros();
  initializeQuaternion();
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
}

void calibrateGyro() {
  double sumX = 0, sumY = 0, sumZ = 0;
  int samples = 1000;
  
  for(int i = 0; i < samples; i++) {
    readMPU();
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(1);
  }
  
  gyroXoffset = sumX / samples;
  gyroYoffset = sumY / samples;
  gyroZoffset = sumZ / samples;
}

void readMPU() {
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 14, true);
  
  ax = (Wire.read() << 8 | Wire.read()) / 16384.0;
  ay = (Wire.read() << 8 | Wire.read()) / 16384.0;
  az = (Wire.read() << 8 | Wire.read()) / 16384.0;
  
  // Skip temperature
  Wire.read(); Wire.read();
  
  gx = (Wire.read() << 8 | Wire.read()) / 131.0;
  gy = (Wire.read() << 8 | Wire.read()) / 131.0;
  gz = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void initializeQuaternion() {
  // Initialize quaternion based on initial accelerometer readings
  readMPU();
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm != 0.0f) {
    ax /= norm;
    ay /= norm;
    az /= norm;
    q0 = sqrt(0.5f * (1 + az));
    q1 = -ay / (2.0f * q0);
    q2 = ax / (2.0f * q0);
    q3 = 0.0f;
  }
}

void updateOrientation(float deltaTime) {
  // Convert gyro values to radians per second
  float gx_rad = (gx - gyroXoffset) * PI / 180.0;
  float gy_rad = (gy - gyroYoffset) * PI / 180.0;
  float gz_rad = (gz - gyroZoffset) * PI / 180.0;
  
  // Madgwick filter update
  madgwickUpdate(gx_rad, gy_rad, gz_rad, ax, ay, az, deltaTime);
  
  // Convert quaternion to Euler angles
  roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  pitch = asin(2 * (q0 * q2 - q3 * q1));
  yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

  // Calculate azimuth and altitude
  azimuth = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  altitude = asin(-2 * (q1 * q3 - q0 * q2));

  // Convert to degrees and normalize
  azimuth = fmod(azimuth * 180 / PI + 360, 360);
  altitude = altitude * 180 / PI;

  // Apply gravity compensation
  compensateGravity();
}

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float deltaTime) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltaTime;
  q1 += qDot2 * deltaTime;
  q2 += qDot3 * deltaTime;
  q3 += qDot4 * deltaTime;

  // Normalise quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void compensateGravity() {
  // Remove gravity component from accelerometer readings
  float gx = 2 * (q1*q3 - q0*q2);
  float gy = 2 * (q0*q1 + q2*q3);
  float gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  
  ax -= gx;
  ay -= gy;
  az -= gz;
}

void loop() {
  readMPU();
  
  unsigned long currentTime = micros();
  double deltaTime = (currentTime - lastUpdateTime) / 1000000.0; // Convert to seconds
  
  // Adaptive timestep
  if (deltaTime > 0.2) {
    deltaTime = 0.2;
  }

  updateOrientation(deltaTime);
  
  lastUpdateTime = currentTime;
  if (millis() - last_ms > OutputInterval){
    StaticJsonDocument<200> doc;
  
    doc["roll"] = roll * 180 / PI;
    doc["pitch"] = pitch * 180 / PI;
    doc["yaw"] = yaw * 180 / PI;
    doc["azimuth"] = azimuth;
    doc["altitude"] = altitude;
  
    // Serialize JSON to Serial
    serializeJson(doc, Serial);
    Serial.println();
    pollButtons();
    last_ms = millis();
  }
}

void pollButtons() {
  // Read the current state of each button
  bool currentButtonState1 = digitalRead(buttonPin1);
  bool currentButtonState2 = digitalRead(buttonPin2);
  
  // Detect button 1 press (transition from HIGH to LOW)
  if (lastButtonState1 == HIGH && currentButtonState1 == LOW) {
    StaticJsonDocument<200> doc;
    doc["btn1"] = true;
    serializeJson(doc, Serial);
    Serial.println();
    button1Pressed = true;
  }
  // Detect button 2 press (transition from HIGH to LOW)
  if (lastButtonState2 == HIGH && currentButtonState2 == LOW) {
    StaticJsonDocument<200> doc;
    doc["btn2"] = true;
    serializeJson(doc, Serial);
    Serial.println();
    button2Pressed = true;
  }
  
  // Update the last known states
  lastButtonState1 = currentButtonState1;
  lastButtonState2 = currentButtonState2;
}
#include<Wire.h>
#include <ArduinoJson.h>
const int MPU_addr1 = 0x68;
float xa, ya, za, gx, gy, gz;
float roll, pitch, yaw = 0;
float gyroZoffset = -9.3;
unsigned long lastUpdateTime;
const float alpha = 1.0; // Complementary filter coefficient

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
}

void calibrateGyro() {
  float sumZ = 0;
  int samples = 1000;
  
  for(int i = 0; i < samples; i++) {
    readMPU();
    sumZ += gz;
    delay(1);
  }
  
  gyroZoffset = sumZ / samples;
  //Serial.print("Gyro Z offset: ");
  //Serial.println(gyroZoffset);
}

void readMPU() {
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 14, true);
  
  xa = (Wire.read() << 8 | Wire.read()) / 16384.0;
  ya = (Wire.read() << 8 | Wire.read()) / 16384.0;
  za = (Wire.read() << 8 | Wire.read()) / 16384.0;
  
  // Skip temperature
  Wire.read(); Wire.read();
  
  gx = (Wire.read() << 8 | Wire.read()) / 131.0;
  gy = (Wire.read() << 8 | Wire.read()) / 131.0;
  gz = (Wire.read() << 8 | Wire.read()) / 131.0;
}

float computeYaw(float gyroZ, float deltaTime) {
  float gyroRate = gyroZ - gyroZoffset;
  float gyroAngle = gyroRate * deltaTime;
  //Serial.print("gyroRate: "+ String(gyroRate) + " gyroAngle: "+ String(gyroAngle) + "\n");
  // Complementary filter
  yaw = alpha * (yaw + gyroAngle);
  
  // Normalize yaw to 0-360 degrees
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  
  return yaw;
}

void loop() {
  readMPU();
  
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastUpdateTime) / 1000000.0; // Convert to seconds
  
  // Adaptive timestep
  if (deltaTime > 0.2) {
    deltaTime = 0.2;
  }
  
  roll = atan2(ya, za) * 180 / PI;
  pitch = -atan2(-xa, sqrt(ya * ya + za * za)) * 180 / PI;
  yaw = computeYaw(gz, deltaTime);
  
  lastUpdateTime = currentTime;

  StaticJsonDocument<200> doc;

  doc["roll"] = roll;
  doc["pitch"] = pitch;
  doc["yaw"] = yaw;

  // Serialize JSON to Serial
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(10);
}
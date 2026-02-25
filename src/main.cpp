#include <Arduino.h>
#include <BLEGamepad.h>
#include <Wire.h>

#ifdef PID
#undef PID
#endif

#include "MPU6050.h"

MPU6050 mpu(0x68);

BleGamepadConfiguration bleGamepadConfig;
BleGamepad bleGamepad("Wiimote V2", "Espressif", 100);
void setup()
{
  bleGamepadConfig.setAutoReport(true);
  bleGamepadConfig.setAxesMin(0);
  bleGamepadConfig.setAxesMax(255);

  bleGamepad.begin(&bleGamepadConfig);

  bleGamepad.setLeftThumb(127, 127);
  bleGamepad.setRightThumb(127, 127);

  Wire.begin(16, 17);
  Wire.setClock(400000);

  Serial.begin(111520);
  while (!Serial)
  {
    delay(10);
  }

  mpu.initialize();

  delay(100);
}

float yaw = 0.0f;
unsigned long lastTime = 0;

void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  // TODO AHRS filter
  lastTime = currentTime;

  if (bleGamepad.isConnected())
  {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int32_t rawGX = gx + 930;
    int32_t rawGY = gy - 480;
    int32_t rawGZ = gz - 40;

    if (abs(rawGX) < 250)
      rawGX = 0;
    if (abs(rawGY) < 250)
      rawGY = 0;
    if (abs(rawGZ) < 250)
      rawGZ = 0;

    u_int8_t joyLX = map(rawGX, -12000, 12000, 0, 255);
    u_int8_t joyLY = map(rawGY, -12000, 12000, 0, 255);
    u_int8_t joyRZ = map(rawGZ, -12000, 12000, 0, 255);

    float pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 57.296;
    float roll = atan2(ay, az) * 57.296;

    u_int8_t joyPitch = map(constrain(pitch, -90, 90), -90, 90, 0, 255);
    u_int8_t joyRoll = map(constrain(roll, -90, 90), -90, 90, 0, 255);

    u_int8_t joyYaw = map(rawGZ, -12000, 12000, 0, 255);

    bleGamepad.setLeftThumb(joyRoll, joyPitch);
    bleGamepad.setRightThumb(joyRZ, joyYaw);

    Serial.print("g/a:\t");
    Serial.print(joyLX);
    Serial.print("\t");
    Serial.print(joyLY);
    Serial.print("\t");
    Serial.print(joyRZ);
    Serial.print("\t\t");
    Serial.print(joyRoll);
    Serial.print("\t");
    Serial.print(joyPitch);
    Serial.print("\t");
    Serial.print(joyYaw);
    Serial.print("\n");
  }
  delay(10);
}
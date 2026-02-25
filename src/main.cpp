#include <Arduino.h>
#include <BLEGamepad.h>
#include <Wire.h>

#ifdef PID
#undef PID
#endif

#include "MPU6050.h"

MPU6050 mpu(0x68);

BleGamepad bleGamepad;
void setup()
{
  bleGamepad = BleGamepad();
  bleGamepad.begin();

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

void loop()
{

  if (bleGamepad.isConnected())
  {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);

    int gamepadGX = map(gx, -17000, 17000, -32767, 32767);
    int gamepadGY = map(gy, -17000, 17000, -32767, 32767);
    int gamepadGZ = map(gz, -17000, 17000, -32767, 32767);
    int gamepadAX = map(ax, -17000, 17000, -32767, 32767);
    int gamepadAY = map(ay, -17000, 17000, -32767, 32767);
    int gamepadAZ = map(az, -17000, 17000, -32767, 32767);

    gamepadGX = constrain(gamepadGX, -32767, 32767);
    gamepadGY = constrain(gamepadGY, -32767, 32767);
    gamepadGZ = constrain(gamepadGZ, -32767, 32767);
    gamepadAX = constrain(gamepadAX, -32767, 32767);
    gamepadAY = constrain(gamepadAY, 0, 32767);
    gamepadAZ = constrain(gamepadAZ, 0, 32767);

    bleGamepad.setLeftThumb(gamepadGX, gamepadGY);
    bleGamepad.setRightThumb(gamepadGZ, gamepadAX);
    bleGamepad.setLeftTrigger(gamepadAY);
    bleGamepad.setRightTrigger(gamepadAZ);
  }

  delay(30);
}

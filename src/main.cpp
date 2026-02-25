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

    // --- 1. GYRO (Speed of movement) ---
    // Offsets applied to keep joysticks at 0 when still
    int joyLX = map(gx + 930, -8000, 8000, -32767, 32767);
    int joyLY = map(gy - 480, -8000, 8000, -32767, 32767);
    int joyRZ = map(gz - 40, -8000, 8000, -32767, 32767);

    // --- 2. ACCELEROMETER (Absolute Tilt) ---
    // We use atan2 to turn raw gravity into an angle (Radians to Degrees)
    // This allows you to know exactly how much you are leaning.
    float pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 57.296;
    float roll = atan2(ay, az) * 57.296;

    // Map angles (-90 to 90 degrees) to the Right Stick Y-axis
    // This is your JoyRY. It tells Godot the exact lean angle.
    int joyRY = map(pitch, -90, 90, -32767, 32767);

    // --- 3. TRIGGER MAPPING ---
    // Using the Z-axis (up/down) for a trigger.
    // When flat, az is 16384. When vertical, az is 0.
    int triggerR = map(az, 0, 16384, 32767, 0);

    // --- 4. APPLY & SEND ---
    bleGamepad.setLeftThumb(constrain(joyLX, -32767, 32767),
                            constrain(joyLY, -32767, 32767));

    bleGamepad.setRightThumb(constrain(joyRZ, -32767, 32767),
                             constrain(joyRY, -32767, 32767));

    bleGamepad.setRightTrigger(constrain(triggerR, 0, 32767));

    Serial.print("g/a:\t");
    Serial.print(joyLX);
    Serial.print("\t");
    Serial.print(joyLY);
    Serial.print("\t");
    Serial.print(joyRZ);
    Serial.print("\t");
    Serial.print(joyRY);
    Serial.print("\t");
    Serial.print(triggerR);
    Serial.print("\n");
  }
  delay(10);
}
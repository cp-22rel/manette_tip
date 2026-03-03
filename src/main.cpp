#include <Arduino.h>
#include <BLEGamepad.h>
#include <Wire.h>

#ifdef PID
#undef PID
#endif

#include "MPU6050.h"

MPU6050 mpu(0x68);

BleGamepadConfiguration bleGamepadConfig;
BleGamepad bleGamepad("Wiimote V2", "Espressif", 67);

const int redPin = 18;
const int greenPin = 23;
const int bluePin = 19;

const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;

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

  pinMode(21, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  ledcSetup(redChannel, 5000, 8);
  ledcSetup(greenPin, 5000, 8);
  ledcSetup(blueChannel, 5000, 8);

  ledcAttachPin(redPin, redChannel);
  ledcAttachPin(greenPin, greenChannel);
  ledcAttachPin(bluePin, blueChannel);

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

u_int8_t controllerState = 0;
bool resetButtonPressed = false;
bool lastResetButtonState = false;

void reset_led();

void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  // TODO AHRS filter
  lastTime = currentTime;

  if (bleGamepad.isConnected())
  {
    if (controllerState == 0)
    {
      controllerState == 1;
      reset_led();
      ledcWrite(blueChannel, 255);
      delay(500);
    }
    controllerState = 2;
    lastResetButtonState = resetButtonPressed;
    resetButtonPressed = digitalRead(21) == LOW;
    bool actionButtonPressed = digitalRead(13) == LOW;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int32_t rawGX = gx + 930;
    int32_t rawGY = gy - 480;
    int32_t rawGZ = gz - 40;

    float gyroZrate = (gz - 40) / 131.0f;

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
    // float pitch = atan2(-ax, sqrt(pow(ax, 2) + pow(az, 2)));
    float roll = atan2(ay, az) * 57.296;
    // float roll = atan2(-ax, az);

    yaw = (yaw + gyroZrate * dt) * 0.999f;

    u_int8_t joyPitch = map(constrain(pitch, -90, 90), -90, 90, 0, 255);
    u_int8_t joyRoll = map(constrain(roll, -90, 90), -90, 90, 0, 255);
    u_int8_t joyYaw = map(constrain(yaw, -180, 180), -180, 180, 0, 255);

    bleGamepad.setLeftThumb(joyRoll, joyPitch);
    bleGamepad.setRightThumb(joyYaw, 128);

    if (resetButtonPressed)
    {
      controllerState = 4;
      bleGamepad.press(BUTTON_5);
    }
    else
    {
      if (lastResetButtonState)
      {
        yaw = 0.0f;
      }
      bleGamepad.release(BUTTON_5);
    }

    if (actionButtonPressed)
    {
      bleGamepad.press(BUTTON_1);
    }
    else
    {
      bleGamepad.release(BUTTON_1);
    }

    Serial.print("State/Buttons/G/RPY:\t");
    Serial.print(controllerState);
    Serial.print("\t\t");
    Serial.print(resetButtonPressed);
    Serial.print("\t");
    Serial.print(actionButtonPressed);
    Serial.print("\t\t");
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
  else
  {
    controllerState = 0;
  }

  if (controllerState == 0)
  {
    reset_led();
    ledcWrite(redChannel, 255);
  }
  else if (controllerState == 1)
  {
    reset_led();
    ledcWrite(blueChannel, 255);
  }
  else if (controllerState == 2)
  {
    reset_led();
    ledcWrite(greenChannel, 255);
  }
  else if (controllerState == 3)
  {
    reset_led();
    delay(500);
    ledcWrite(redChannel, 255);
    delay(500);
  }
  else if (controllerState == 4)
  {
    reset_led();
    ledcWrite(redChannel, 128);
    ledcWrite(blueChannel, 128);
  }

  delay(10);
}

void reset_led()
{
  ledcWrite(redChannel, 0);
  ledcWrite(greenChannel, 0);
  ledcWrite(blueChannel, 0);
}

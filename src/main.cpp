#include <Arduino.h>
#include <driver/rtc_io.h>
#include <BLEGamepad.h>
#include <Wire.h>

#ifdef PID
#undef PID
#endif

#include "MPU6050.h"

MPU6050 mpu(0x68);

BleGamepadConfiguration bleGamepadConfig;
BleGamepad bleGamepad("Wiimote V2", "Espressif", 67);

const bool debug = false;

const int redPin = 18;
const int greenPin = 23;
const int bluePin = 19;

const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;

float offset_ax = 0.0f;
float offset_ay = 0.0f;
float offset_az = 0.0f;

float gravity = 8192.0f;

void reset_led();
void handle_gamepad_led_state();
double moving_average(double avg, double newSample, double n);

// SETUP -------------------------------------
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

  uint64_t mask = 1ULL << GPIO_NUM_13;
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);

  Serial.begin(111520);
  while (!Serial)
  {
    delay(10);
  }

  mpu.initialize();

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  delay(100);

  Serial.println("\nCalibrating MPU... DO NOT MOVE CONTROLLER!");

  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  int cycles = 200;
  for (int i = 0; i < cycles; i++)
  {
    int16_t c_ax, c_ay, c_az, c_gx, c_gy, c_gz;

    mpu.getMotion6(&c_ax, &c_ay, &c_az, &c_gx, &c_gy, &c_gz);
    sum_ax += c_ax;
    sum_ay += c_ay;
    sum_az += c_az;
    delay(4);
  }

  offset_ax = sum_ax / cycles;
  offset_ay = sum_ay / cycles;
  offset_az = (sum_az / cycles) - gravity;

  Serial.println("Calibration complete!");
}

// LOOP VARIABLE -------------------------------

float yaw = 0.0f;
float acc_saved_roll = 0.0f;
float acc_saved_picht = 0.0f;

float vel_x = 0, vel_y = 0, vel_z = 0;
float pos_x = 0, pos_y = 0, pos_z = 0;

unsigned long lastTime = 0;

u_int8_t controllerState = 0;
bool resetButtonPressed = false;
bool lastResetButtonState = false;

double pitch_avg = 0;
double roll_avg = 0;

float unconnected_time = 0;
const float time_until_deep_sleep = 10;

// LOOP ----------------------------------------
void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;

  lastTime = currentTime;

  if (bleGamepad.isConnected())
  {
    unconnected_time = 0;

    if (controllerState == 0)
    {
      controllerState == 1;
      reset_led();
      ledcWrite(blueChannel, 255);
      delay(500);
    }
    controllerState = 2;

    resetButtonPressed = digitalRead(21) == LOW;
    bool actionButtonPressed = digitalRead(13) == LOW;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int32_t rawGX = gx + 930;
    int32_t rawGY = gy - 480;
    int32_t rawGZ = gz - 40;

    int32_t rawAX = ax - offset_ax;
    int32_t rawAY = ay - offset_ay;
    int32_t rawAZ = az - offset_az;

    float gyroZrate = (gz - 40) / 131.0f;
    float gyroYrate = (gy - 480) / 131.0f;
    float gyroXrate = (gx + 930) / 131.0f;

    if (abs(rawGX) < 250)
      rawGX = 0;
    if (abs(rawGY) < 250)
      rawGY = 0;
    if (abs(rawGZ) < 250)
      rawGZ = 0;

    u_int8_t joyLX = map(rawGX, -12000, 12000, 0, 255);
    u_int8_t joyLY = map(rawGY, -12000, 12000, 0, 255);
    u_int8_t joyRZ = map(rawGZ, -12000, 12000, 0, 255);

    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.296;
    float roll = atan2(ay, az) * 57.296;

    yaw = (yaw + gyroZrate * dt) * 0.999f;

    // Acceleration Positionning

    float ax_g = ax / gravity;
    float ay_g = ay / gravity;
    float az_g = az / gravity;

    float pitch_for_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g));
    float roll_for_acc = atan2(ay_g, az_g);

    pitch_avg = moving_average(pitch_avg, pitch_for_acc, 3);
    roll_avg = moving_average(roll_avg, roll_for_acc, 3);

    pitch_for_acc = pitch_avg;
    roll_for_acc = roll_avg;

    float alpha = 0.01;

    acc_saved_picht = alpha * (acc_saved_picht + gyroXrate * dt) + (1 - alpha) * pitch_for_acc;
    acc_saved_roll = alpha * (acc_saved_roll + gyroYrate * dt) + (1 - alpha) * roll_for_acc;

    pitch_for_acc = acc_saved_picht;
    roll_for_acc = acc_saved_roll;

    float gravity_x = gravity * -sin(pitch_for_acc);
    float gravity_y = gravity * cos(pitch_for_acc) * sin(roll_for_acc);
    float gravity_z = gravity * cos(pitch_for_acc) * cos(roll_for_acc);

    float linear_ax = ax - gravity_x;
    float linear_ay = ay - gravity_y;
    float linear_az = az - gravity_z - 700;

    linear_ax = (abs(linear_ax) > 50) * linear_ax;
    linear_ay = (abs(linear_ay) > 50) * linear_ay;
    linear_az = (abs(linear_az) > 50) * linear_az;

    float range = 400.0f;
    float velocityLimit = 200;

    vel_x = constrain(vel_x + linear_ax * dt, -velocityLimit, velocityLimit);
    vel_y = constrain(vel_y + linear_ay * dt, -velocityLimit, velocityLimit);
    vel_z = constrain(vel_z + linear_az * dt, -velocityLimit, velocityLimit);

    vel_x = (abs(vel_x) > 9) * vel_x;
    vel_y = (abs(vel_y) > 9) * vel_y;
    vel_z = (abs(vel_z) > 9) * vel_z;

    pos_x = constrain(pos_x + vel_x, -range, range);
    pos_y = constrain(pos_y + vel_y, -range, range);
    pos_z = constrain(pos_z + vel_z, -range, range);

    vel_x = vel_x * 0.4;
    vel_y = vel_y * 0.4;
    vel_z = vel_z * 0.4;

    pos_x = pos_x * 0.8;
    pos_y = pos_y * 0.8;
    pos_z = pos_z * 0.8;

    u_int8_t joyPosX = map(pos_x, -range, range, 0, 255);
    u_int8_t joyPosY = map(pos_y, -range, range, 0, 255);
    u_int8_t joyPosZ = map(pos_z, -range, range, 0, 255);

    u_int8_t joyPitch = map(constrain(pitch, -90, 90), -90, 90, 0, 255);
    u_int8_t joyRoll = map(constrain(roll, -90, 90), -90, 90, 0, 255);
    u_int8_t joyYaw = map(constrain(yaw, -180, 180), -180, 180, 0, 255);

    bleGamepad.setLeftThumb(joyRoll, joyPitch);
    bleGamepad.setRightThumb(joyYaw, joyPosX);
    bleGamepad.setLeftTrigger(joyPosY);
    bleGamepad.setRightTrigger(joyPosZ);

    if ((!resetButtonPressed || !actionButtonPressed) && lastResetButtonState)
    {
      lastResetButtonState = false;
      yaw = 0.0f;
      pos_x = 0;
      pos_y = 0;
      pos_z = 0;
      vel_x = 0;
      vel_y = 0;
      vel_z = 0;
      controllerState = 4;
    }
    else if (resetButtonPressed && actionButtonPressed)
    {
      lastResetButtonState = true;
      bleGamepad.press(BUTTON_5);
      bleGamepad.release(BUTTON_1);
      bleGamepad.release(BUTTON_2);
    }
    else
    {
      bleGamepad.release(BUTTON_5);

      if (resetButtonPressed)
      {
        bleGamepad.press(BUTTON_2);
      }
      else
      {
        bleGamepad.release(BUTTON_2);
      }

      if (actionButtonPressed)
      {
        bleGamepad.press(BUTTON_1);
      }
      else
      {
        bleGamepad.release(BUTTON_1);
      }
    }
    if (debug)
    {
      Serial.print(">");
      Serial.print("state:");
      Serial.print(controllerState);
      Serial.print(",");
      Serial.print("reset_btn:");
      Serial.print(resetButtonPressed);
      Serial.print(",");
      Serial.print("action_btn:");
      Serial.print(actionButtonPressed);
      Serial.print(",");
      Serial.print("gx:");
      Serial.print(joyLX);
      Serial.print(",");
      Serial.print("gy:");
      Serial.print(joyLY);
      Serial.print(",");
      Serial.print("gz:");
      Serial.print(joyRZ);
      Serial.print(",");
      Serial.print("roll:");
      Serial.print(joyRoll);
      Serial.print(",");
      Serial.print("pitch:");
      Serial.print(joyPitch);
      Serial.print(",");
      Serial.print("yaw:");
      Serial.print(joyYaw);
      Serial.print(",");
      Serial.print("pos_x:");
      Serial.print(pos_x);
      Serial.print(",");
      Serial.print("pos_y:");
      Serial.print(pos_y);
      Serial.print(",");
      Serial.print("pos_z:");
      Serial.print(pos_z);
      Serial.print(",");
      Serial.print("gx:");
      Serial.print(rawGX);
      Serial.print(",");
      Serial.print("gy:");
      Serial.print(rawGY);
      Serial.print(",");
      Serial.print("gz:");
      Serial.print(rawGZ);
      Serial.print(",");
      Serial.print("vel_x:");
      Serial.print(vel_x);
      Serial.print(",");
      Serial.print("vel_y:");
      Serial.print(vel_y);
      Serial.print(",");
      Serial.print("vel_z:");
      Serial.print(vel_z);
      Serial.print(",");
      Serial.print("gravity_x:");
      Serial.print(gravity_x);
      Serial.print(",");
      Serial.print("gravity_y:");
      Serial.print(gravity_y);
      Serial.print(",");
      Serial.print("gravity_z:");
      Serial.print(gravity_z);
      Serial.print(",");
      Serial.print("linear_ax:");
      Serial.print(linear_ax);
      Serial.print(",");
      Serial.print("linear_ay:");
      Serial.print(linear_ay);
      Serial.print(",");
      Serial.print("linear_az:");
      Serial.print(linear_az);
      Serial.print("\r\n");
    }
  }
  else
  {
    unconnected_time += dt;
    controllerState = 0;

    if (unconnected_time >= time_until_deep_sleep)
    {
      esp_deep_sleep_start();
    }
  }

  handle_gamepad_led_state();

  delay(10);
}

// METHODS------------------------------

void reset_led()
{
  ledcWrite(redChannel, 0);
  ledcWrite(greenChannel, 0);
  ledcWrite(blueChannel, 0);
}

void handle_gamepad_led_state()
{
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
    delay(100);
  }
}

double moving_average(double avg, double newSample, double n)
{
  avg -= avg / n;
  avg += newSample / n;
  return avg;
}
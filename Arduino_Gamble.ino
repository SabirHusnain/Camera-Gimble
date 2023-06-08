#include <Wire.h>
// Header file for PCA9685 driver
#include <Adafruit_PWMServoDriver.h>

// Servo Driver Pulse Width and Frequency
#define MIN_PULSE_WIDTH 85
#define MAX_PULSE_WIDTH 495
#define frequency 50

#define servoA 0
#define servoB 1
#define servoC 2

// MPU6050 sensor Data
#define adrs 0x68
#define accMaxRange 4.0
#define accSensitivity 8192.0
#define gyroMaxRange 500.0
#define gyroSensitivity 65.5

// Variables
long accX, accY, accZ, accVect;
long gyroX, gyroY, gyroZ;
float rollGyro, pitchGyro, yawGyro;
float rollAcc, pitchAcc;
float rollAngle, pitchAngle, yawAngle;
long gyroErrorX, gyroErrorY, gyroErrorZ;
long loopTime;
bool setInitialAngles = false;

// Functions
void moveMotor(int, int);
void config6050();
void readGyro();
void readAcc();
void calibrateGyro();

// Initilization if PWM for driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(115200);
  Wire.begin();

  config6050();
  calibrateGyro();

  loopTime = micros();
}

void loop()
{
  readGyro();
  readAcc();

  gyroX -= gyroErrorX;
  gyroY -= gyroErrorY;
  gyroZ -= gyroErrorZ;

  pitchGyro += gyroX * 0.00006107; // 0.00006107 = 1/250 Hz/geroSensitivity
  rollGyro += gyroY * 0.00006107;
  yawGyro += gyroZ * 0.00006107;

  pitchGyro += rollGyro * sin(gyroZ * 0.000001065); // 0.000001065 = (1/250 Hz/geroSensitivity)*(Pi/180)
  rollGyro -= pitchGyro * sin(gyroZ * 0.000001065);

  accVect = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

  pitchAcc = asin((float)accY / accVect) * 57.29578; // 57.29578 = 180/Pi
  rollAcc = -asin((float)accX / accVect) * 57.29578;

  if (setInitialAngles)
  {
    // Using 99.96% of Gyro and 0.04% of Accelerometer
    pitchGyro = pitchGyro * 0.9996 + pitchAcc * 0.0004;
    rollGyro = rollGyro * 0.9996 + rollAcc * 0.0004;
  }
  else
  {
    pitchGyro = pitchAcc;
    rollGyro = rollAcc;
    setInitialAngles = true;
  }
  // Filter to filter the ourput data for smooth running of sensor
  pitchAngle = pitchAngle * 0.9 + pitchGyro * 0.1;
  rollAngle = rollAngle * 0.9 + rollGyro * 0.1;
  yawAngle = yawGyro;

  Serial.print(pitchAngle);
  Serial.print(" , ");
  Serial.print(rollAngle);
  Serial.print(" , ");
  Serial.println(yawAngle);

  moveMotor(0, 90.0 - pitchAngle);
  moveMotor(1, 90.0 - rollAngle);
  moveMotor(2, 90.0 - yawAngle);

  //loop until timer reaches 4000uS which is 250Hz
  while (micros() - loopTime < 4000);
  loopTime = micros();
}

void moveMotor(int motorOut, int angle)
{
  int pulse_wide, pulse_width;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * frequency * 4096);
  pwm.setPWM(motorOut, 0, pulse_wide);
}

void config6050()
{
  Wire.beginTransmission(adrs);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();

  Wire.beginTransmission(adrs);
  Wire.write(0x1C);
  Wire.write(0b00001000);
  Wire.endTransmission();

  Wire.beginTransmission(adrs);
  Wire.write(0x1B);
  Wire.write(0b00001000);
  Wire.endTransmission();
}

void readGyro()
{
  Wire.beginTransmission(adrs);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(adrs, 6);
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void readAcc()
{
  Wire.beginTransmission(adrs);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(adrs, 6);
  while (Wire.available() < 6);
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
}

void calibrateGyro()
{
  for (int i = 0; i < 2000; i++)
  {
    readGyro();
    gyroErrorX = gyroErrorX + gyroX;
    gyroErrorY = gyroErrorY + gyroY;
    gyroErrorZ = gyroErrorZ + gyroZ;
  }

  gyroErrorX = gyroErrorX / 2000;
  gyroErrorY = gyroErrorY / 2000;
  gyroErrorZ = gyroErrorZ / 2000;
}

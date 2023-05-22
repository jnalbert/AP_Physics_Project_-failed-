// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <list>
// using namespace std;

// #include "helper_functions.h"

// Adafruit_MPU6050 mpu;

// // global variables
// float accelOffsetsGlobal[3];
// float gyroOffsetsGlobal[3];
// float anglesGlobal[3];
// float positionGlobal[3];
// float velocityGlobal[3];

// void setup()
// {

//   initalSetUp();
//   calibrateMPU6050Mean(mpu, accelOffsetsGlobal, gyroOffsetsGlobal);
// }

// void initalSetUp()
// {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

//   // Try to initialize!
//   if (!mpu.begin())
//   {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1)
//     {
//       delay(10);
//     }
//   }

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);

//   Serial.println("Stabilizing sensor...");
//   delay(5000); // Allow time for sensor stabilization
// }

// void loop()
// {

//   if (Serial.available())
//   {
//     // Message received, read the data
//     char receivedChar = Serial.read();

//     // Process the received character or message
//     // Example: Print the received character
//     if (receivedChar == 'c')
//     {
//       Serial.print("Old ones");
//       printOffsets(accelOffsetsGlobal);
//       calibrateMPU6050Mean(mpu, accelOffsetsGlobal, gyroOffsetsGlobal);
//       Serial.print("New Ones");
//       printOffsets(accelOffsetsGlobal);
//     }
//     else if (receivedChar == 'm')
//     {
//       Serial.print("Old ones");
//       printOffsets(accelOffsetsGlobal);
//       calibrateMPU6050Median(mpu, accelOffsetsGlobal, gyroOffsetsGlobal);
//       Serial.print("New Ones");
//       printOffsets(accelOffsetsGlobal);
//     }
//   }
//   else
//   {
//     getSensorDataByMedian(mpu, 20, 1, accelOffsetsGlobal, gyroOffsetsGlobal);
//     // getPositionDataComplementary(mpu, 20, 1, accelOffsetsGlobal, gyroOffsetsGlobal, anglesGlobal, positionGlobal, velocityGlobal);
//   }
// }




// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <list>
// using namespace std;
// #include "helper_functions.h"
// #include <MadgwickAHRS.h>
// #include <MahonyAHRS.h>

// #define UPDATE_INTERVAL_MS 10  // Update interval in milliseconds


// Adafruit_MPU6050 mpu;
// Madgwick filter;
// Mahony fusionFilter;

// float initialPositionX = 0.0;  // Initial X position
// float initialPositionY = 0.0;  // Initial Y position
// float initialPositionZ = 0.0;  // Initial Z position

// float initialYaw = 0.0;    // Initial yaw angle in radians
// float initialPitch = 0.0;  // Initial pitch angle in radians
// float initialRoll = 0.0;   // Initial roll angle in radians

// float velocityX = 0.0;  // Current X velocity
// float velocityY = 0.0;  // Current Y velocity
// float velocityZ = 0.0;  // Current Z velocity

// unsigned long lastUpdateTime = 0;

// // global variables
// float accelOffsetsGlobal[3];
// float gyroOffsetsGlobal[3];
// float anglesGlobal[3];
// float positionGlobal[3];
// float velocityGlobal[3];

// void setup()
// {

//   initalSetUp();
//   calibrateMPU6050Mean(mpu, accelOffsetsGlobal, gyroOffsetsGlobal);

//   filter.begin(UPDATE_INTERVAL_MS / 1000.0);
// }

// void initalSetUp()
// {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

//   // Try to initialize!
//   if (!mpu.begin())
//   {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1)
//     {
//       delay(10);
//     }
//   }

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);

//   Serial.println("Stabilizing sensor...");
//   delay(5000); // Allow time for sensor stabilization
// }

// void loop()
// {
//   unsigned long currentTime = millis();
//   std::map<std::string, double> sensorDataWithOffsets = getSensorDataByMedian(mpu, 10, 1, accelOffsetsGlobal, gyroOffsetsGlobal);

//    // Convert accelerometer data to meters per second squared (m/s^2)
//   float ax = sensorDataWithOffsets["accelX"];
//   float ay = sensorDataWithOffsets["accelY"];
//   float az = sensorDataWithOffsets["accelZ"] + 9.81;
  
//   // Convert gyroscope data to radians per second (rad/s)
//   float gx = sensorDataWithOffsets["gyroX"];
//   float gy = sensorDataWithOffsets["gyroY"];
//   float gz = sensorDataWithOffsets["gyroZ"];
  
//   // Update the filters with the new data
//   filter.updateIMU(gx, gy, gz, ax, ay, az);
//   fusionFilter.updateIMU(gx, gy, gz, ax, ay, az);
  
//   // Calculate the elapsed time since the last update
//   float dt = (currentTime - lastUpdateTime) / 1000.0;
//   lastUpdateTime = currentTime;
  
//   // Get the estimated orientation quaternion from Madgwick filter
  
//   // Calculate the yaw angle in radians from Madgwick filter
//   float yaw = filter.getYawRadians();
  
//   // Get the estimated orientation quaternion from Mahony filter
//   float fw, fx, fy, fz;
//   fusionFilter.getQuaternion(&fw, &fx, &fy, &fz);
  
//   // Calculate the yaw angle in radians from Mahony filter
//   float fusedYaw = atan2(2.0 * (fw * fz + fx * fy), 1.0 - 2.0 * (fy * fy + fz * fz));
  
//   // Calculate the change in yaw angle
//   float deltaYaw = fusedYaw - initialYaw;
  
//   // Calculate the displacement in X and Y directions using dead reckoning
//   float deltaX = cos(initialYaw) * gx * dt;
//   float deltaY = sin(initialYaw) * gx * dt;
  
//   // Update the initial position and yaw angle
//   initialX += deltaX;
//   initialY += deltaY;
//   initialYaw = fusedYaw;
  
//   // Print the current position
//   Serial.print("X: ");
//   Serial.print(initialX);
//   Serial.print("\tY: ");
//   Serial.println(initialY);
  
//   delay(UPDATE_INTERVAL_MS);
// }




#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <list>
using namespace std;
#include "helper_functions.h"
#include <MadgwickAHRS.h>


#define UPDATE_INTERVAL_MS 10  // Update interval in milliseconds

Adafruit_MPU6050 mpu;
Madgwick filter;

float initialPositionX = 0.0;  // Initial X position
float initialPositionY = 0.0;  // Initial Y position
float initialPositionZ = 0.0;  // Initial Z position

float initialYaw = 0.0;    // Initial yaw angle in radians
float initialPitch = 0.0;  // Initial pitch angle in radians
float initialRoll = 0.0;   // Initial roll angle in radians

float velocityX = 0.0;  // Current X velocity
float velocityY = 0.0;  // Current Y velocity
float velocityZ = 0.0;  // Current Z velocity

unsigned long lastUpdateTime = 0;
// global variables
float accelOffsetsGlobal[3];
float gyroOffsetsGlobal[3];
float anglesGlobal[3];
float positionGlobal[3];
float velocityGlobal[3];

void setup()
{

  initalSetUp();
  calibrateMPU6050Mean(mpu, accelOffsetsGlobal, gyroOffsetsGlobal);

  float beta = 0.1;  // Adjust this value based on your application
  filter.begin(UPDATE_INTERVAL_MS / 1000.0);
}

void initalSetUp()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  Serial.println("Stabilizing sensor...");
  delay(5000); // Allow time for sensor stabilization

}

void loop() {
  unsigned long currentTime = millis();

  // Read accelerometer and gyroscope data
std::map<std::string, double> sensorDataWithOffsets = getSensorDataByMedian(mpu, 10, 1, accelOffsetsGlobal, gyroOffsetsGlobal);

  // Convert accelerometer data to meters per second squared (m/s^2)
  float ax = sensorDataWithOffsets["accelX"];
  float ay = sensorDataWithOffsets["accelY"];
  float az = sensorDataWithOffsets["accelZ"] + 9.81;

  // Convert gyroscope data to radians per second (rad/s)
  float gx = sensorDataWithOffsets["gyroX"];
  float gy = sensorDataWithOffsets["gyroY"];
  float gz = sensorDataWithOffsets["gyroZ"];

  // Update the filter with the new data
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // Calculate the elapsed time since the last update

  float yaw = filter.getYawRadians();
  float pitch = filter.getPitchRadians();
  float roll = filter.getRollRadians();

  
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tRoll: ");
  Serial.print(roll);
  Serial.print("\tYaw: ");
  Serial.println(yaw);

}

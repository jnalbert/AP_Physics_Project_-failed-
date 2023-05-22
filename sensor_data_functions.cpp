#include "helper_functions.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <list>
#include <map>
using namespace std;

// std::map<string, float>
std::map<std::string, double> getSensorDataByMedian(Adafruit_MPU6050 &mpuRef, int numSamples, int timeDelay, float accelOffsets[3], float gyroOffsets[3]) {

  list<float> xDataAccel, yDataAccel, zDataAccel, xDataGyro, yDataGyro, zDataGyro;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpuRef.getEvent(&a, &g, &temp);

    a.acceleration.x -= accelOffsets[0];
    a.acceleration.y -= accelOffsets[1];
    a.acceleration.z -= accelOffsets[2];
    xDataAccel.push_back(a.acceleration.x);
    yDataAccel.push_back(a.acceleration.y);
    zDataAccel.push_back(a.acceleration.z);

    a.gyro.x -= gyroOffsets[0];
    a.gyro.y -= gyroOffsets[1];
    a.gyro.z -= gyroOffsets[2];
    xDataGyro.push_back(a.gyro.x);
    yDataGyro.push_back(a.gyro.y);
    zDataGyro.push_back(a.gyro.z);
    delay(timeDelay);
    }
    xDataAccel.sort();
    yDataAccel.sort();
    zDataAccel.sort();
    xDataGyro.sort();
    yDataGyro.sort();
    zDataGyro.sort();

    list<float>::iterator xItAccel = xDataAccel.begin();
    advance(xItAccel, numSamples / 2);
    list<float>::iterator yItAccel = yDataAccel.begin();
    advance(yItAccel, numSamples / 2);
    list<float>::iterator zItAccel = zDataAccel.begin();
    advance(zItAccel, numSamples / 2);

    list<float>::iterator xItGyro = xDataGyro.begin();
    advance(xItGyro, numSamples / 2);
    list<float>::iterator yItGyro = yDataGyro.begin();
    advance(yItGyro, numSamples / 2);
    list<float>::iterator zItGyro = zDataGyro.begin();
    advance(zItGyro, numSamples / 2);

    // printAccelValues(*xItAccel, *yItAccel, *zItAccel);
    // printGyroValues(*xItGyro, *yItGyro, *zItGyro);
    // return a map of acceleration and gryo values
    return {
      {"accelX", *xItAccel},
      {"accelY", *yItAccel},
      {"accelZ", *zItAccel},
      {"gyroX", *xItGyro},
      {"gyroY", *yItGyro},
      {"gyroZ", *zItGyro}
    };
}

void printAccelValues(float x, float y, float z) {
  Serial.print("Accel_X:");
  Serial.print(x);
  Serial.print(",");
  Serial.print("Accel_Y:");
  Serial.print(y);
  Serial.print(",");
  Serial.print("Accel_Z:");
  Serial.println(z);

  // Serial.print("Acceleration [X,Y,Z]: ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.print(y);
  // Serial.print(", ");
  // Serial.print(z);
  // Serial.println(" m/s^2");
}

void printGyroValues(float x, float y, float z) {
  Serial.print("Gyro_X:");
  Serial.print(x);
  Serial.print(",");
  Serial.print("Gyro_Y:");
  Serial.print(y);
  Serial.print(",");
  Serial.print("Gyro_Z:");
  Serial.println(z);

}

void printOffsets (float offsets[3]) {
  Serial.print(" X offset: ");
  Serial.print(offsets[0]);
  Serial.print("  Y offset: ");
  Serial.print(offsets[1]);
  Serial.print("  Z offset: ");
  Serial.println(offsets[2]);
}

void getPositionDataComplementary(Adafruit_MPU6050 &mpuRef, int numSamples, int timeDelay, float accelOffsets[3], float gyroOffsets[3], float angles[3], float position[3], float velocity[3]) {
  const float errorMargin = 0.05;  // Error margin for acceleration offset calibration
  const float dt = (numSamples * timeDelay) / 1000.0;  // Time step in seconds
  // Serial.print("dt: ");
  // Serial.println(dt);
  const float alpha = 0.98;  // Complementary filter constant

  float accX, accY, accZ;  // Acceleration values
  float gyroX, gyroY, gyroZ;  // Gyroscope values
  std::map<std::string, double> sensorDataWithOffsets = getSensorDataByMedian(mpuRef, numSamples, timeDelay, accelOffsets, gyroOffsets);

  // loop through the sensorDataWithOffsets map and check if the value is within the error margin and then set it to 0 if it is
  accZ += 9.81;  // Add gravity to acceleration values
  for (auto& x : sensorDataWithOffsets)
  {
    if (abs(x.second) < errorMargin) {
      x.second = 0;
    }
  }
  // printAccelValues(sensorDataWithOffsets["accelX"], sensorDataWithOffsets["accelY"], sensorDataWithOffsets["accelZ"]);
  // printGyroValues(sensorDataWithOffsets["gyroX"], sensorDataWithOffsets["gyroY"], sensorDataWithOffsets["gyroZ"]);

  // convert m/s^2 to g
  accX = sensorDataWithOffsets["accelX"] / 9.81;
  accY = sensorDataWithOffsets["accelY"] / 9.81;
  accZ = sensorDataWithOffsets["accelZ"] / 9.81;
  gyroX = sensorDataWithOffsets["gyroX"] * DEG_TO_RAD;
  gyroY = sensorDataWithOffsets["gyroY"] * DEG_TO_RAD;
  gyroZ = sensorDataWithOffsets["gyroZ"] * DEG_TO_RAD;

  // Complementary filter for roll and pitch estimation
  float accAngleX = atan2(accY, accZ) * RAD_TO_DEG;  // Calculate acceleration-based angle around X-axis
  float accAngleY = atan2(accX, accZ) * RAD_TO_DEG;  // Calculate acceleration-based angle around Y-axis
  float accAngleZ = atan2(accX, accY) * RAD_TO_DEG;  // Calculate acceleration-based angle around Z-axis

  angles[0] = alpha * (angles[0] + gyroX * dt) + (1 - alpha) * accAngleX;  // Complementary filter update for X-axis
  angles[1] = alpha * (angles[1] + gyroY * dt) + (1 - alpha) * accAngleY;  // Complementary filter update for Y-axis
  angles[2] = alpha * (angles[2] + gyroZ * dt) + (1 - alpha) * accAngleZ;  // Complementary filter update for Z-axis

  // Integrate acceleration to obtain velocity
  velocity[0] += (accX * cos(angles[1]) - accY * sin(angles[1])) * dt;
  velocity[1] += (accX * sin(angles[0]) * sin(angles[1]) + accY * cos(angles[0]) + accZ * sin(angles[0]) * cos(angles[1])) * dt;

  // Integrate velocity to obtain position
  position[0] += velocity[0] * dt;
  position[1] += velocity[1] * dt;
  position[2] += velocity[2] * dt;

  // Print estimated position
  Serial.print("Pos_X: ");
  Serial.print(position[0]);
  Serial.print(", ");
  Serial.print("Pos_Y: ");
  Serial.print(position[1]);
  Serial.print(", ");
  Serial.print("Pos_Z: ");
  Serial.println(position[2]);

}

void getPositionDataKalmanFilter(Adafruit_MPU6050 &mpuRef, int numSamples, int timeDelay, float accelOffsets[3], float gyroOffsets[3], float angles[2], float position[2], float velocity[2]) {

}

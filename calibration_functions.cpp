#include "helper_functions.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <list>
using namespace std;


void calibrateMPU6050Mean(Adafruit_MPU6050 &mpuRef, float accelOffsetsRef[3], float gyroOffsetsRef[3]) {
  Serial.println("Calibrating MPU6050. Please keep the sensor stationary (Mean)");
  const int numSamples = 1000;
  const int sampleDelay = 4;

  float accelOffsets[3] = {0.0f, 0.0f, 0.0f};
  float gyroOffsets[3] = {0.0f, 0.0f, 0.0f};

  // Collect accelerometer and gyroscope data for calibration
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpuRef.getEvent(&a, &g, &temp);

    accelOffsets[0] += a.acceleration.x;
    accelOffsets[1] += a.acceleration.y;
    accelOffsets[2] += a.acceleration.z;

    gyroOffsets[0] += g.gyro.x;
    gyroOffsets[1] += g.gyro.y;
    gyroOffsets[2] += g.gyro.z;

    delay(sampleDelay);
  }

  // Calculate the average offsets
  accelOffsets[0] /= numSamples;
  accelOffsets[1] /= numSamples;
  accelOffsets[2] /= numSamples;

  gyroOffsets[0] /= numSamples;
  gyroOffsets[1] /= numSamples;
  gyroOffsets[2] /= numSamples;

  // prints the offsets
  // sets thhe accel offsets
  accelOffsetsRef[0] = accelOffsets[0];
  accelOffsetsRef[1] = accelOffsets[1];
  accelOffsetsRef[2] = accelOffsets[2];

  // set the gryo offsets
  gyroOffsetsRef[0] = gyroOffsets[0];
  gyroOffsetsRef[1] = gyroOffsets[1];
  gyroOffsetsRef[2] = gyroOffsets[2];


  // Set the calculated offsets
  // mpu.setXAccelOffset(-accelOffsets[0]);
  // mpu.setYAccelOffset(-accelOffsets[1]);
  // mpu.setZAccelOffset((16384 - accelOffsets[2]));

  // mpu.setXGyroOffset(-gyroOffsets[0]);
  // mpu.setYGyroOffset(-gyroOffsets[1]);
  // mpu.setZGyroOffset(-gyroOffsets[2]);
  Serial.println("Calibration complete. (mean)");
}

void calibrateMPU6050Median(Adafruit_MPU6050 &mpuRef, float accelOffsetsRef[3], float gyroOffsetsRef[3]) {
  Serial.println("Calibrating MPU6050. Please keep the sensor stationary (Median)");
  const int numSamples = 1000;
  const int sampleDelay = 4;

  list<float> xDataAccel, yDataAccel, zDataAccel;
  list<float> xDataGryo, yDataGryo, zDataGryo;

  // Collect accelerometer and gyroscope data for calibration
  for (int i = 0; i < numSamples; i++) {
    if (i % 100 == 0) Serial.println(i);
    sensors_event_t a, g, temp;
    mpuRef.getEvent(&a, &g, &temp);

    xDataAccel.push_back(a.acceleration.x);
    yDataAccel.push_back(a.acceleration.y);
    zDataAccel.push_back(a.acceleration.z);

    xDataGryo.push_back(g.gyro.x);
    yDataGryo.push_back(g.gyro.y);
    zDataGryo.push_back(g.gyro.z);

    delay(sampleDelay);
  }
  Serial.print("Got here");

  // sort the data
  xDataAccel.sort();
  yDataAccel.sort();
  zDataAccel.sort();
  xDataGryo.sort();
  yDataGryo.sort();
  zDataGryo.sort();

  // get the middle value
  list<float>::iterator xItAccel = xDataAccel.begin();
  advance(xItAccel, numSamples / 2);
  list<float>::iterator yItAccel = yDataAccel.begin();
  advance(yItAccel, numSamples / 2);
  list<float>::iterator zItAccel = zDataAccel.begin();
  advance(zItAccel, numSamples / 2);

  list<float>::iterator xItGryo = xDataGryo.begin();
  advance(xItGryo, numSamples / 2);
  list<float>::iterator yItGryo = yDataGryo.begin();
  advance(yItGryo, numSamples / 2);
  list<float>::iterator zItGryo = zDataGryo.begin();
  advance(zItGryo, numSamples / 2);

  // sets thhe accel offsets
  accelOffsetsRef[0] = *xItAccel;
  accelOffsetsRef[1] = *yItAccel;
  accelOffsetsRef[2] = *zItAccel;

  // set the gryo offsets
  gyroOffsetsRef[0] = *xItGryo;
  gyroOffsetsRef[1] = *yItGryo;
  gyroOffsetsRef[2] = *zItGryo;

  Serial.println("Calibration complete. (mean)");
}
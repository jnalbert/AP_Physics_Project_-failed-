// helper_functions.h
#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <map>

void testPrint(); // Function declaration
void calibrateMPU6050Mean(Adafruit_MPU6050 &mpuRef, float accelOffsetsRef[3], float gyroOffsetsRef[3]);
void calibrateMPU6050Median(Adafruit_MPU6050 &mpuRef, float accelOffsetsRef[3], float gyroOffsetsRef[3]);
std::map<std::string, double> getSensorDataByMedian(Adafruit_MPU6050 &mpuRef, int numSamples, int timeDelay, float accelOffsets[3], float gyroOffsets[3]);
void printAccelValues(float x, float y, float z);
void printGyroValues(float x, float y, float z);
void printOffsets (float offsets[3]);
void getPositionDataComplementary(Adafruit_MPU6050 &mpuRef, int numSamples, int timeDelay, float accelOffsets[3], float gyroOffsets[3], float angles[3], float position[3], float velocity[3]);

#endif
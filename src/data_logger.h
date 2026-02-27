//pragma makes it so data_logger.h is compiled once
#pragma once
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

//Depending on what you want to do 
//comment and uncomment what you want to do
//#define REAL_FLIGHT

//this makes it so if REAL_FLIGHT is defined the controller
//isn't wasting time trying to print serial data
#ifndef REAL_FLIGHT
#define SERIAL
#endif

//SD_log can be one at anytime for real flight or testing
#define SD_LOG


//Serial functions
void print_serial_acceleration(float x, float y, float z);
void print_serial_magneticfield(float x, float y, float z);
void print_serial_gyroscope(float x, float y, float z);
void print_serial_temperature(float T);
void print_serial_altitude(float H);
void print_serial_pressure(float P);

//SD card functions

int appendLog(char* chunk_buf, int chunk_index, int CHUNK_SIZE,
float ax, float ay, float az, float gx, float gy, float gz, float H);
int serviceSD(bool* chunk_ready, int chunk_index, 
  File &logFile, char* chunk_buf);

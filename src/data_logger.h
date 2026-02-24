//pragma makes it so data_logger.h is compiled once
#pragma once
#include <Arduino.h>

//Depending on what you want to do 
//comment and uncomment what you want to do
#define SERIAL_TEST
//#define SD_LOG
//#define REAL_FLIGHT

//Serial functions
void print_serial_acceleration(float x, float y, float z);
void print_serial_magneticfield(float x, float y, float z);
void print_serial_gyroscope(float x, float y, float z);
void print_serial_temperature(float T);
void print_serial_altitude(float H);
void print_serial_pressure(float P);

//SD card functions
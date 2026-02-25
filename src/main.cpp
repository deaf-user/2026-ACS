#include "data_logger.h"
#include "read_accel_data.h"

//make global variables
//object for grabbing sensor data
ExternalSensors sensors;
//variables to hold sensor data
//T = temperature, H = height, P = pressure
static float x,y,z,T,H,P;

#ifdef REAL_FLIGHT
enum states state = resting;
//endif for REAL_FLIGHT
#endif

#ifdef SD_LOG
#include <SD.h>
#include <SPI.h>
#endif

const int chipSelect = BUILTIN_SDCARD;

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }

  // open the file.
  

  // if the file is available, write to it:
  
}



// #include "../lib/eigen/Eigen/Eigen"

void setup() {

    // these statements print serial data to the laptop
    // SERIAL_TEST is defined in data_logger.h
    #ifdef SERIAL
    //instantiates serial port to communicate
    Serial.begin(115200);
    //gives computer time to open serial moniter
    delay(10000);   // give USB time to enumerate
    Serial.print("Serial ready");
    //Since serial is define startupTasks() spits data out
    sensors.startupTasks();
    //endif for SERIAL_TEST
    #endif

    #ifdef REAL_FLIGHT

    sensors.startupTasks();
    //endif for REAL_Flight
    #endif

    #ifdef SD_LOG
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1) {
        // No SD card, so don't do anything more - stay stuck here
        }
    }

    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    #endif
}

//hi nathan

void loop(void){

    //1 HZ sampling period for accelerometer
    #ifdef SERIAL
    //change for different sampling frequencies
    delay(200);
    sensors.readAcceleration(x,y,z);
    print_serial_acceleration(x,y,z);
    sensors.readGyroscope(x,y,z);
    print_serial_gyroscope(x,y,z);
    //sensors.readTemperature(T);
    //print_serial_temperature(T);
    sensors.readAltitude(H);
    print_serial_altitude(H);
    //sensors.readPressure(P);
    //print_serial_pressure(P);
    //endif for SERIAL_TEST
    #endif

    #ifdef REAL_FLIGHT
    switch(state):
        //grabs accel_data
        //grabs height data
        case RESTING:
        // position is higher than initial position
        // switch state if 100 ft higher than initial position
        // switch to SPEED_UP
        // wait an initial time for safety of start up transients
        break;
        case SPEED_UP:
        // tracks acceleration
        // once acceleration is less than 0
        // some value switch states
        break;
        case MACH_FLIGHT:
        // checks velocity
        // if velocity is above .7 dont move
        // if velocity is consistently below .7
        // switch
        break;
        case CONTROL:
        // implements the control loop
        // moves if next altitude is lower than previous altitude
        break;
        case RECOVERY:
        // do nothing
        break;
        
    //endif for REAL_FLIGHT
    #endif
}
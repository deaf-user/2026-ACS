#include "data_logger.h"
#include "read_accel_data.h"


//make global variables
//object for grabbing sensor data
ExternalSensors sensors;
//variables to hold sensor data
//T = temperature, H = height, P = pressure
static float x,y,z,T,H,P;

void setup() {
    // these statements print serial data to the laptop
    // SERIAL_TEST is defined in data_logger.h
    #ifdef SERIAL_TEST
    //instantiates serial port to communicate
    Serial.begin(115200);
    //gives computer time to open serial moniter
    delay(10000);   // give USB time to enumerate
    Serial.print("Serial ready");
    //Since serial is define startupTasks() spits data out
    sensors.startupTasks();
    //endif for SERIAL_TEST
    #endif
}

void loop(void){

    //1 HZ sampling period for accelerometer
    #ifdef SERIAL_TEST
    //change for different sampling frequencies
    delay(200);
    sensors.readAcceleration(x,y,z);
    print_serial_acceleration(x,y,z);
    //sensors.readMagneticField(x,y,z);
    //print_serial_magneticfield(x,y,z);
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
}
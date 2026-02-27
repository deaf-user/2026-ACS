#include "data_logger.h"
#include "read_accel_data.h"
#include <IntervalTimer.h>

IntervalTimer controlTimer;

volatile bool RUN_ACS = false;

//make global variables
//object for grabbing sensor data
ExternalSensors sensors;

//variables to hold sensor data
//H = height
// a_variable are the accelerometer data
// g_variable are the gyroscope data
// Ka or Kg are filtered data
static float ax,ay,az,H,gx,gy,gz;
static float kax,kay,kaz,kH,kgx,kgy,kgz;

// presently dont need these
//T = temperature, P = pressure
//static float T,P;

#ifdef REAL_FLIGHT
enum states state = RESTING;
//endif for REAL_FLIGHT
#endif

#ifdef SD_LOG
#include <SD.h>
#include <SPI.h>
#define CHUNK_SIZE 4096

File Raw_data;
File Filtered_data;

char chunk_buf_raw[CHUNK_SIZE];
char chunk_buf_filtered[CHUNK_SIZE];
volatile int chunk_idx_raw = 0;
volatile int chunk_idx_filtered = 0;
bool chunk_ready_raw = false;
bool chunk_ready_filtered = false;

#endif

const int chipSelect = BUILTIN_SDCARD;

void controlISR(){
  RUN_ACS = true;
}

void setup() {

    // 150 Hz = 6667 microseconds
    controlTimer.begin(controlISR, 6667);

    // these statements print serial data to the laptop
    // SERIAL is defined in data_logger.h
    #ifdef SERIAL
    //instantiates serial port to communicate
    Serial.begin(115200);
    //gives computer time to open serial moniter
    delay(10000);   // give USB time to enumerate
    Serial.print("Serial ready");
    #endif

    #ifdef SD_LOG
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1) {
        // No SD card, so don't do anything more - stay stuck here
        }
    }
    Raw_data = SD.open("Raw_data.bin", FILE_WRITE);
    Filtered_data = SD.open("Filtered_data.bin", FILE_WRITE);
    #endif

    sensors.startupTasks();
}

//hi nathan

void loop(void){
  
  if (RUN_ACS) {
        RUN_ACS = false;
        ACS_update();
  }

  chunk_idx_raw = serviceSD( &chunk_ready_raw, chunk_idx_raw, Raw_data, chunk_buf_raw);
  chunk_idx_filtered = serviceSD( &chunk_ready_filtered, chunk_idx_filtered, Filtered_data, chunk_buf_filtered);

}

void ACS_update(void){

  sensors.readAcceleration(ax,ay,az);
  sensors.readGyroscope(gx,gy,gz);
  sensors.readAltitude(H);

  #ifdef SERIAL
  print_serial_acceleration(ax,ay,az);
  print_serial_gyroscope(gx,gy,gz);
  print_serial_altitude(H);
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
  #endif

  #ifdef SD_LOG
    chunk_idx_raw = appendLog(chunk_buf_raw, chunk_idx_raw, CHUNK_SIZE,
    ax, ay, az, gx, gy, gz, H);
    if(chunk_idx_raw >= CHUNK_SIZE - 256){
      chunk_ready_raw = true;
    }
    chunk_idx_filtered = appendLog(chunk_buf_filtered, chunk_idx_filtered, CHUNK_SIZE,
    kax, kay, kaz, kgx, kgy, kgz, kH);
    if(chunk_idx_filtered >= CHUNK_SIZE -256 ){
      chunk_ready_filtered = true;
    }
  #endif
}
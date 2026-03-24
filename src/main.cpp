#include "data_logger.h"
#include "read_accel_data.h"
#include "Quaternion.h"
#include <IntervalTimer.h>
#include <Servo.h>

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
#define RESTING 0
#define SPEED_UP 1
#define MACH_FLIGHT 2
#define CONTROL 3
#define RECOVERY 4
int state = RESTING;
static float init_H = 0;

Servo myServo_darling;
int servo_end_us = 1500;
int servo_start_us = 1835;

void controlISR(){
  RUN_ACS = true;
}
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
const int chipSelect = BUILTIN_SDCARD;

void ACS_update(void);

#ifdef SERIAL
#undef SERIAL
#endif

#define ACCEL_THRESHOLLD 4.66f

uint32_t t0 = 0;

#endif

#ifdef TEST_ANGLE
Quaternion Quat;
static float roll, pitch, yaw;
#endif

void setup() {

    // these statements print serial data to the laptop
    // SERIAL is defined in data_logger.h
    #ifdef SERIAL
    //instantiates serial port to communicate
    Serial.begin(115200);
    //gives computer time to open serial moniter
    delay(10000);   // give USB time to enumerate
    Serial.print("Serial ready\n");
    #endif

    #ifdef SD_LOG
    // if (!SD.begin(chipSelect)) {
    //     #ifdef SERIAL
    //     Serial.println("Card failed, or not present");
    //     #endif
        
    //     while (1) {
    //     // No SD card, so don't do anything more - stay stuck here
    //     }
    // }

    // if(SD.exists("Raw_data.txt"))
    //   SD.remove("Raw_data.txt");
    
    Raw_data = SD.open("Raw_data.txt", FILE_WRITE);
    Raw_data.seek(Raw_data.size());
    // Filtered_data = SD.open("Filtered_data.bin", FILE_WRITE);
    #endif

    sensors.startupTasks();

    #ifdef REAL_FLIGHT
    // 150 Hz = 6667 microseconds
    controlTimer.begin(controlISR, 6667);
    sensors.readAltitude(init_H);
    myServo_darling.attach(3);
    myServo_darling.writeMicroseconds(servo_start_us);
    #endif

    #ifdef TEST_ANGLE
    sensors.readGyroscope(roll, pitch, yaw);
    #endif
}

//hi nathan

void loop(void){
  
  
  #ifdef TEST_ANGLE
  static int delay_time = 100;
  static float Ts = (float)delay_time/1000;

  sensors.readAcceleration(ax,ay,az);
  sensors.readGyroscope(gx,gy,gz);

  imu_update(&Quat, gx,gy,gz,ax,ay,az,Ts);
  quat_to_euler(Quat, &roll,&pitch,&yaw);

  
  // my attempt
  // delay(delay_time);
  // static float alpha = 0.98f;
  // sensors.readGyroscope(gx,gy,gz);
  // sensors.readAcceleration(ax,ay,az);
  // // current_gx += gx*Ts;
  // // current_gy += gy*Ts;
  // static float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az));
  // static float accel_roll  = atan2(ay, az);
  // current_gy = alpha * (current_gy + gy * Ts)
  //     + (1 - alpha) * accel_pitch;
  // current_gx  = alpha * (current_gx  + gx * Ts)
  //     + (1 - alpha) * accel_roll;  
  // current_gz += gz*Ts;



  print_serial_gyroscope(roll, pitch, yaw);
  // print_serial_gyroscope(gx, gy, gz);
  #endif

  #ifdef REAL_FLIGHT
  if (RUN_ACS) {
        RUN_ACS = false;
        ACS_update();
  }
  chunk_idx_raw = serviceSD( &chunk_ready_raw, chunk_idx_raw, Raw_data, chunk_buf_raw);
  chunk_idx_filtered = serviceSD( &chunk_ready_filtered, chunk_idx_filtered, Filtered_data, chunk_buf_filtered);
  #endif

}

void ACS_update(void){

  static int ACS_counter = 0;
  static int start_debounce = 0;
  static int debounce_counter = 0;
  static int debounce_done = 0;
  ACS_counter++;
  sensors.readAcceleration(ax,ay,az);
  sensors.readGyroscope(gx,gy,gz);
  sensors.readAltitude(H);

  #ifdef SERIAL
  // print_serial_acceleration(ax,ay,az);
  // print_serial_gyroscope(gx,gy,gz);
  // print_serial_altitude(H);
  
  //didn't want to remake the function
  //using this now.
  // print_serial_altitude(state);
  #endif

  #ifdef REAL_FLIGHT
  switch(state) {
    case RESTING:
      myServo_darling.writeMicroseconds(servo_start_us);
      if(az > ACCEL_THRESHOLLD) {
        start_debounce = 1;
      } 

      if(start_debounce == 1){
        debounce_counter++;
        if(debounce_counter == (150 * 1)){
          debounce_done = 1;
        }
      }
      
      if(debounce_done == 1){
        start_debounce = 0;
        debounce_done = 0;
        debounce_counter = 0;
        state = SPEED_UP;
        t0 = micros();
      }
    break;

    case SPEED_UP:
      myServo_darling.writeMicroseconds(servo_start_us);
      if(az < ACCEL_THRESHOLLD) {
        start_debounce = 1;
      } 

      if(start_debounce == 1){
        debounce_counter++;
        if(debounce_counter == (150 * 1.5)){
          debounce_done = 1;
        }
      }
      
      if(debounce_done == 1){
        start_debounce = 0;
        debounce_done = 0;
        debounce_counter = 0;
        state = MACH_FLIGHT;
        ACS_counter = 0;
      }

    break;

    case MACH_FLIGHT:
      myServo_darling.writeMicroseconds(servo_start_us);
      if(ACS_counter > (150 * 2)){
        state = CONTROL;
        myServo_darling.writeMicroseconds(servo_end_us);
        ACS_counter = 0;
      }
      
    break;
    case CONTROL:
      myServo_darling.writeMicroseconds(servo_end_us);
      if(ACS_counter > (150 * 60 * 2)){
        state = RECOVERY;
        myServo_darling.writeMicroseconds(servo_start_us);
      }
    // implements the control loop
    // moves if next altitude is lower than previous altitude
    break;
    case RECOVERY:
    // do nothing
    break;
  }
  #endif

  #ifdef SD_LOG
  if(state >= SPEED_UP && state < RECOVERY){
    chunk_idx_raw = appendLog(chunk_buf_raw, chunk_idx_raw, CHUNK_SIZE,
    ax, ay, az, gx, gy, gz, H);
    if(chunk_idx_raw >= CHUNK_SIZE - 256){
      chunk_ready_raw = true;
    }
  }else if(state >= RECOVERY){
      chunk_idx_raw = serviceSD( &chunk_ready_raw, chunk_idx_raw, Raw_data, chunk_buf_raw);
      Raw_data.flush();
      Raw_data.close();
  }

    // chunk_idx_filtered = appendLog(chunk_buf_filtered, chunk_idx_filtered, CHUNK_SIZE,
    // kax, kay, kaz, kgx, kgy, kgz, kH);
    // if(chunk_idx_filtered >= CHUNK_SIZE -256 ){
    //   chunk_ready_filtered = true;
    // }
  #endif
}
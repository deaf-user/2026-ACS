#include "data_logger.h"

void print_serial_acceleration(float x, float y, float z){
  static char buf[7];
  //sensors.readAcceleration(x,y,z);
  dtostrf(x,-7,3,buf);
  Serial.print("X Accel = ");
  Serial.print(buf);
  dtostrf(y,-7,3,buf);
  Serial.print("  Y Accel = ");
  Serial.print(buf);
  dtostrf(z,-7,3,buf);
  Serial.print("  Z Accel = ");
  Serial.print(buf);
  Serial.print("\n");
  Serial.write('\r');
}

void print_serial_magneticfield(float x, float y, float z){
  static char buf[7];
  //sensors.readAcceleration(x,y,z);
  dtostrf(x,-7,3,buf);
  Serial.print("X Mag = ");
  Serial.print(buf);
  dtostrf(y,-7,3,buf);
  Serial.print("  Y Mag = ");
  Serial.print(buf);
  dtostrf(z,-7,3,buf);
  Serial.print("  Z Mag = ");
  Serial.print(buf);
  Serial.print("\n");
  Serial.write('\r');
}

void print_serial_gyroscope(float x, float y, float z){
  static char buf[7];
  //sensors.readAcceleration(x,y,z);
  dtostrf(x,-7,3,buf);
  Serial.print("Roll = ");
  Serial.print(buf);
  dtostrf(y,-7,3,buf);
  Serial.print("  Pitch = ");
  Serial.print(buf);
  dtostrf(z,-7,3,buf);
  Serial.print("  Yaw = ");
  Serial.print(buf);
  Serial.print("\n");
  Serial.write('\r');
}

void print_serial_temperature(float T){
    static char buf[7];
    dtostrf(T,-7,3,buf);
    Serial.print("temperature = ");
    Serial.print(buf);
    Serial.print("\n");
    Serial.write('\r');
}

void print_serial_altitude(float H){
    static char buf[7];
    dtostrf(H,-7,3,buf);
    Serial.print("Height = ");
    Serial.print(buf);
    Serial.print("\n");
    Serial.write('\r');
}

void print_serial_pressure(float P){
    static char buf[7];
    dtostrf(P,-7,3,buf);
    Serial.print("Pressure = ");
    Serial.print(buf);
    Serial.print("\n");
    Serial.write('\r');
}

int appendLog(char* chunk_buf, int chunk_index, int CHUNK_SIZE,
float ax, float ay, float az, float gx, float gy, float gz, float H)
{
    int n = snprintf(
        &chunk_buf[chunk_index],
        CHUNK_SIZE - chunk_index,
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        micros(),
        ax, ay, az,
        gx, gy, gz, 
        H);

    chunk_index += n;
    return (chunk_index);
}

int serviceSD(bool* chunk_ready, int chunk_index, 
  File &logFile, char* chunk_buf)
{
    if (*chunk_ready)
    {
        noInterrupts();

        uint32_t bytesToWrite = chunk_index;
        chunk_index = 0;
        *chunk_ready = false;

        interrupts();

        logFile.write((uint8_t*)chunk_buf, bytesToWrite);
    }
    return (chunk_index);
}
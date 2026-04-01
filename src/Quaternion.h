#include <math.h>

#define DEG2RAD (3.14159265f / 180.0f)

typedef struct {
    float w, x, y, z;
} Quaternion;

void normalize_quaternion(Quaternion* q);

void imu_update(Quaternion* q,
                float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt);

void quat_to_euler(Quaternion q, 
    float* roll, float* pitch, float* yaw);
#include "Quaternion.h"

void normalize_quaternion(Quaternion* q)
{
    float norm = sqrtf(q->w*q->w + q->x*q->x +
                       q->y*q->y + q->z*q->z);
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

void imu_update(Quaternion* q,
                float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt)
{
    // Normalize accel
    gx *=  DEG2RAD;
    gy *=  DEG2RAD;
    gz *=  DEG2RAD;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated gravity from quaternion
    float vx = 2*(q->x*q->z - q->w*q->y);
    float vy = 2*(q->w*q->x + q->y*q->z);
    float vz = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;

    // Error (cross product)
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // Gain (tune this)
    float Kp = 10.0f;

    // Correct gyro
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;

    // Quaternion derivative
    float qw = q->w;
    float qx = q->x;
    float qy = q->y;
    float qz = q->z;

    q->w += (-qx*gx - qy*gy - qz*gz) * 0.5f * dt;
    q->x += ( qw*gx + qy*gz - qz*gy) * 0.5f * dt;
    q->y += ( qw*gy - qx*gz + qz*gx) * 0.5f * dt;
    q->z += ( qw*gz + qx*gy - qy*gx) * 0.5f * dt;

    normalize_quaternion(q);
}

void quat_to_euler(Quaternion q, float* roll, float* pitch, float* yaw)
{
    *roll = atan2f(2*(q.w*q.x + q.y*q.z),
                   1 - 2*(q.x*q.x + q.y*q.y));

    *pitch = asinf(2*(q.w*q.y - q.z*q.x));

    *yaw = atan2f(2*(q.w*q.z + q.x*q.y),
                  1 - 2*(q.y*q.y + q.z*q.z));
}
#include "quaternion_filter.h"

#include <math.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f

static void QuaternionFilter_PropagateRad(
    QuaternionFilter *filter,
    float wx,
    float wy,
    float wz,
    float dt_s
)
{
    // Save the old quaternion so every derivative uses the same state
    const float qw = filter->qw;
    const float qx = filter->qx;
    const float qy = filter->qy;
    const float qz = filter->qz;

    // q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
    const float qw_dot =
        0.5f * (-qx * wx - qy * wy - qz * wz);

    const float qx_dot =
        0.5f * (qw * wx + qy * wz - qz * wy);

    const float qy_dot =
        0.5f * (qw * wy - qx * wz + qz * wx);

    const float qz_dot =
        0.5f * (qw * wz + qx * wy - qy * wx);

    // Euler integration
    filter->qw = qw + qw_dot * dt_s;
    filter->qx = qx + qx_dot * dt_s;
    filter->qy = qy + qy_dot * dt_s;
    filter->qz = qz + qz_dot * dt_s;

    // Keep the quaternion at unit length
    const float norm = sqrtf(
        filter->qw * filter->qw +
        filter->qx * filter->qx +
        filter->qy * filter->qy +
        filter->qz * filter->qz
    );

    if (norm > 0.0f)
    {
        const float inverse_norm = 1.0f / norm;

        filter->qw *= inverse_norm;
        filter->qx *= inverse_norm;
        filter->qy *= inverse_norm;
        filter->qz *= inverse_norm;
    }
}

void QuaternionFilter_Init(QuaternionFilter *filter)
{
    // Identity orientation: no rotation
    filter->qw = 1.0f;
    filter->qx = 0.0f;
    filter->qy = 0.0f;
    filter->qz = 0.0f;
}

void QuaternionFilter_UpdateGyro(
    QuaternionFilter *filter,
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float dt_s
)
{
    // Convert angular velocity from degrees/s to radians/s
    const float wx = gx_dps * DEG_TO_RAD;
    const float wy = gy_dps * DEG_TO_RAD;
    const float wz = gz_dps * DEG_TO_RAD;

    QuaternionFilter_PropagateRad(
        filter,
        wx,
        wy,
        wz,
        dt_s
    );
}

void QuaternionFilter_UpdateMahony(
    QuaternionFilter *filter,
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float ax_g,
    float ay_g,
    float az_g,
    float kp,
    float dt_s
)
{
    // Convert angular velocity from degrees/s to radians/s
    float wx = gx_dps * DEG_TO_RAD;
    float wy = gy_dps * DEG_TO_RAD;
    float wz = gz_dps * DEG_TO_RAD;

    // Normalize the accelerometer direction
    const float accel_norm = sqrtf(
        ax_g * ax_g +
        ay_g * ay_g +
        az_g * az_g
    );

    if (accel_norm > 0.0f)
    {
        const float inverse_accel_norm = 1.0f / accel_norm;

        const float ax = ax_g * inverse_accel_norm;
        const float ay = ay_g * inverse_accel_norm;
        const float az = az_g * inverse_accel_norm;

        const float qw = filter->qw;
        const float qx = filter->qx;
        const float qy = filter->qy;
        const float qz = filter->qz;

        // Predicted gravity/up direction expressed in body coordinates
        const float vx =
            2.0f * (qx * qz - qw * qy);

        const float vy =
            2.0f * (qw * qx + qy * qz);

        const float vz =
            qw * qw - qx * qx - qy * qy + qz * qz;

        // Orientation error: measured direction × predicted direction
        const float error_x = ay * vz - az * vy;
        const float error_y = az * vx - ax * vz;
        const float error_z = ax * vy - ay * vx;

        // Feed the gravity-direction error back into the gyro rates
        wx += kp * error_x;
        wy += kp * error_y;
        wz += kp * error_z;
    }

    // Propagate the quaternion using the corrected angular velocity
    QuaternionFilter_PropagateRad(
        filter,
        wx,
        wy,
        wz,
        dt_s
    );
}

void QuaternionFilter_ToEuler(
    const QuaternionFilter *filter,
    float *roll_deg,
    float *pitch_deg,
    float *yaw_deg
)
{
    const float qw = filter->qw;
    const float qx = filter->qx;
    const float qy = filter->qy;
    const float qz = filter->qz;

    const float roll_rad = atan2f(
        2.0f * (qw * qx + qy * qz),
        1.0f - 2.0f * (qx * qx + qy * qy)
    );

    float pitch_input =
        2.0f * (qw * qy - qz * qx);

    // Prevent slight floating-point error from entering asinf outside [-1, 1]
    if (pitch_input > 1.0f)
    {
        pitch_input = 1.0f;
    }
    else if (pitch_input < -1.0f)
    {
        pitch_input = -1.0f;
    }

    const float pitch_rad = asinf(pitch_input);

    const float yaw_rad = atan2f(
        2.0f * (qw * qz + qx * qy),
        1.0f - 2.0f * (qy * qy + qz * qz)
    );

    *roll_deg = roll_rad * RAD_TO_DEG;
    *pitch_deg = pitch_rad * RAD_TO_DEG;
    *yaw_deg = yaw_rad * RAD_TO_DEG;
}

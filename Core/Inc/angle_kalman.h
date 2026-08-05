#ifndef ANGLE_KALMAN_H
#define ANGLE_KALMAN_H

typedef struct
{
    float angle_deg;
    float bias_dps;

    float p00;
    float p01;
    float p10;
    float p11;

    float q_angle;
    float q_bias;
    float r_measure;
    float accel_rejection_gain;
} AngleKalmanFilter;

void AngleKalman_Init(
    AngleKalmanFilter *filter,
    float initial_angle_deg,
    float q_angle,
    float q_bias,
    float r_measure,
    float accel_rejection_gain
);

float AngleKalman_Update(
    AngleKalmanFilter *filter,
    float accel_angle_deg,
    float gyro_rate_dps,
    float accel_magnitude_g,
    float dt_s
);
#endif

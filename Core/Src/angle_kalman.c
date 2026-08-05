#include "angle_kalman.h"
#include <math.h>

void AngleKalman_Init(
    AngleKalmanFilter *filter,
    float initial_angle_deg,
    float q_angle,
    float q_bias,
    float r_measure,
    float accel_rejection_gain
){
    filter->angle_deg = initial_angle_deg;
    filter->bias_dps = 0.0f;

    filter->p00 = 1.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 1.0f;

    filter->q_angle = q_angle;
    filter->q_bias = q_bias;
    filter->r_measure = r_measure;
    filter->accel_rejection_gain = accel_rejection_gain;
}

float AngleKalman_Update(
    AngleKalmanFilter *filter,
    float accel_angle_deg,
    float gyro_rate_dps,
    float accel_magnitude_g,
    float dt_s
){
    // State prediction
    filter->angle_deg +=
        dt_s * (gyro_rate_dps - filter->bias_dps);

    // Covariance prediction
    float p00_pred =
        filter->p00
        - dt_s * filter->p01
        - dt_s * filter->p10
        + dt_s * dt_s * filter->p11
        + filter->q_angle * dt_s;

    float p01_pred =
        filter->p01
        - dt_s * filter->p11;

    float p10_pred =
        filter->p10
        - dt_s * filter->p11;

    float p11_pred =
        filter->p11
        + filter->q_bias * dt_s;

    // Measurement residual
    float residual =
        accel_angle_deg - filter->angle_deg;

    // Adaptive accelerometer measurement covariance
    float accel_deviation =
        fabsf(accel_magnitude_g - 1.0f);

    float r_current =
        filter->r_measure
        * (1.0f
           + filter->accel_rejection_gain
           * accel_deviation
           * accel_deviation);

    // Innovation covariance
    float innovation_covariance =
        p00_pred + r_current;

    // Kalman gain
    float k_angle =
        p00_pred / innovation_covariance;

    float k_bias =
        p10_pred / innovation_covariance;

    // State correction
    filter->angle_deg +=
        k_angle * residual;

    filter->bias_dps +=
        k_bias * residual;

    // Covariance correction
    filter->p00 =
        p00_pred - k_angle * p00_pred;

    filter->p01 =
        p01_pred - k_angle * p01_pred;

    filter->p10 =
        p10_pred - k_bias * p00_pred;

    filter->p11 =
        p11_pred - k_bias * p01_pred;

    return filter->angle_deg;
}

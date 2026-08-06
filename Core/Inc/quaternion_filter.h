#ifndef INC_QUATERNION_FILTER_H_
#define INC_QUATERNION_FILTER_H_

typedef struct
{
    float qw;
    float qx;
    float qy;
    float qz;
} QuaternionFilter;

void QuaternionFilter_Init(QuaternionFilter *filter);

void QuaternionFilter_UpdateGyro(
    QuaternionFilter *filter,
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float dt_s
);

void QuaternionFilter_ToEuler(
    const QuaternionFilter *filter,
    float *roll_deg,
    float *pitch_deg,
    float *yaw_deg
);

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
);

#endif /* INC_QUATERNION_FILTER_H_ */

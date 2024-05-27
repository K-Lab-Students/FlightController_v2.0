#include "ahrs.hpp"
#include "param/param.hpp"

namespace AHRS
{
    float gyroscopeOffset[] = {-0.00852211565, -0.00426105782, -0.0138484379};
    PARAM_ADD(param::FLOAT, CAL_GYRO_XOFF, &gyroscopeOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_YOFF, &gyroscopeOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_ZOFF, &gyroscopeOffset[2]);

    float accelerometerOffset[] = {0, 0, 0};
    PARAM_ADD(param::FLOAT, CAL_ACC_XOFF, &accelerometerOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_ACC_YOFF, &accelerometerOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_ACC_ZOFF, &accelerometerOffset[2]);

    float accelerationFilterGain = 0.01;
    PARAM_ADD(param::FLOAT, AHRS_ACC_GAIN, &accelerationFilterGain);

    float accelerationRejection = 0.5;
    PARAM_ADD(param::FLOAT, AHRS_ACC_RJT, &accelerationRejection);

    float accelerometerNoise = 0.5;
    PARAM_ADD(param::FLOAT, AHRS_ACC_NOISE, &accelerometerNoise);

    float barometerNoise = 2;
    PARAM_ADD(param::FLOAT, AHRS_BARO_NOISE, &barometerNoise);

    float mulka = 1;
    PARAM_ADD(param::FLOAT, AHRS_EKF_Q, &mulka);

    Eigen::Matrix3f Q{
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 1},
    };
    float q1 = 0, q2 = 0, q3 = 1;
    void s()
    {
        Q(0, 0) = q1;
        Q(1, 1) = q2;
        Q(2, 2) = q3;
    }
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, AHRS_POS_Q, &q1, s);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, AHRS_SPD_Q, &q2, s);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, AHRS_ACC_Q, &q3, s);
}
#include "ahrs.hpp"
#include "atmosphere.hpp"
#include "Math/math.hpp"

#include "Common.hpp"
#include <algorithm>

namespace AHRS
{
    extern float gyroscopeOffset[3];
    extern float accelerometerOffset[3];

    extern float accelerationFilterGain;
    extern float accelerationRejection;

    extern float accelerometerNoise;
    extern float barometerNoise;
    extern float mulka;

    float lastDt = 0;

    Eigen::Vector3f rawRotateSpeed, rotateSpeed;
    Eigen::Vector3f rawAcceleration, acceleration;
    float pressure, temperature;

    Eigen::Quaternionf attitude = Eigen::Quaternionf::Identity();
    Eulerf eulerAttitude;
    float G = 1;

    Eigen::Vector3f linearAcceleration;

    float pressFilter = 0;

    Eigen::Matrix3f P{
        {100, 0, 0},
        {0, 100, 0},
        {0, 0, 10},
    };
    extern Eigen::Matrix3f Q;
    Eigen::Vector3f x{0, 0, 0};

    void predictKalman(const float dt)
    {
        const Eigen::Vector3f newX{
            x[0] + (x[1] * dt) + (x[2] * dt * dt / 2),
            x[1] + (dt * x[2]),
            x[2],
        };
        x = newX;

        const Eigen::Matrix3f newP{
            {
                P(0, 0) + P(1, 0) * dt + ((dt * dt) * ((P(2, 2) * (dt * dt)) / 2 + P(1, 2) * dt + P(0, 2))) / 2 + (P(2, 0) * (dt * dt)) / 2 + dt * ((P(2, 1) * (dt * dt)) / 2 + P(1, 1) * dt + P(0, 1)),
                P(0, 1) + P(1, 1) * dt + (P(2, 1) * (dt * dt)) / 2 + dt * ((P(2, 2) * (dt * dt)) / 2 + P(1, 2) * dt + P(0, 2)),
                (P(2, 2) * (dt * dt)) / 2 + P(1, 2) * dt + P(0, 2),
            },

            {
                P(1, 0) + P(2, 0) * dt + dt * (P(1, 1) + P(2, 1) * dt) + ((dt * dt) * (P(1, 2) + P(2, 2) * dt)) / 2,
                P(1, 1) + P(2, 1) * dt + dt * (P(1, 2) + P(2, 2) * dt),
                P(1, 2) + P(2, 2) * dt,
            },

            {
                (P(2, 2) * (dt * dt)) / 2 + P(2, 1) * dt + P(2, 0),
                P(2, 1) + P(2, 2) * dt,
                P(2, 2),
            },
        };
        P = newP + (Q * (mulka * dt));
    }

    void correctPos(float z,
                    const float R)
    {
        pressFilter = (1 - 0.01) * pressFilter + 0.01 * z;
        z = pressFilter;

        const float S = P(0, 0) + R;
        const float y = x(0) - z;

        Eigen::Vector3f newX{
            x[0] - (P(0, 0) * y) / S,
            x[1] - (P(1, 0) * y) / S,
            x[2] - (P(2, 0) * y) / S,
        };
        x = newX;

        Eigen::Matrix3f newP{
            {
                -P(0, 0) * (P(0, 0) / S - 1),
                -P(0, 1) * (P(0, 0) / S - 1),
                -P(0, 2) * (P(0, 0) / S - 1),
            },
            {
                P(1, 0) - (P(0, 0) * P(1, 0)) / S,
                P(1, 1) - (P(0, 1) * P(1, 0)) / S,
                P(1, 2) - (P(0, 2) * P(1, 0)) / S,
            },
            {
                P(2, 0) - (P(0, 0) * P(2, 0)) / S,
                P(2, 1) - (P(0, 1) * P(2, 0)) / S,
                P(2, 2) - (P(0, 2) * P(2, 0)) / S,
            },
        };
        P = newP;
    }

    void correctAcc(const float z,
                    const float R)
    {
        const float S = P(2, 2) + R;
        const float y = x(2) - z;

        Eigen::Vector3f newX{
            x[0] - (P(0, 2) * y) / S,
            x[1] - (P(1, 2) * y) / S,
            x[2] - (P(2, 2) * y) / S,
        };
        x = newX;

        Eigen::Matrix3f newP{
            {
                P(0, 0) - (P(0, 2) * P(2, 0)) / S,
                P(0, 1) - (P(0, 2) * P(2, 1)) / S,
                P(0, 2) - (P(0, 2) * P(2, 2)) / S,
            },
            {
                P(1, 0) - (P(1, 2) * P(2, 0)) / S,
                P(1, 1) - (P(1, 2) * P(2, 1)) / S,
                P(1, 2) - (P(1, 2) * P(2, 2)) / S,
            },
            {
                -P(2, 0) * (P(2, 2) / S - 1),
                -P(2, 1) * (P(2, 2) / S - 1),
                -P(2, 2) * (P(2, 2) / S - 1),
            },
        };
        P = newP;
    }

    void update()
    {
        const Eigen::Quaternionf attitude = getFRD_Attitude();

        float sinr_cosp = 2 * (attitude.w() * attitude.x() + attitude.y() * attitude.z());
        float cosr_cosp = 1 - 2 * (attitude.x() * attitude.x() + attitude.y() * attitude.y());
        eulerAttitude.roll = std::atan2(sinr_cosp, cosr_cosp);

        float sinp = 2 * (attitude.w() * attitude.y() - attitude.x() * attitude.z());
        if (std::abs(sinp) >= 1)
            eulerAttitude.pitch = std::copysign<float>(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            eulerAttitude.pitch = std::asin(sinp);

        float siny_cosp = 2 * (attitude.w() * attitude.z() + attitude.x() * attitude.y());
        float cosy_cosp = 1 - 2 * (attitude.y() * attitude.y() + attitude.z() * attitude.z());
        eulerAttitude.yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void updateByIMU(Eigen::Vector3f rSpeed, Eigen::Vector3f rAcc, float dT)
    {
        lastDt = dT;
        rawRotateSpeed = rSpeed;
        rawAcceleration = rAcc;

        rotateSpeed = rSpeed - Eigen::Vector3f{gyroscopeOffset};
        acceleration = rAcc - Eigen::Vector3f{accelerometerOffset};

        Eigen::Quaternionf current = attitude;
        current.coeffs() += omega(current, rotateSpeed).coeffs() * dT;
        current.normalize();
        attitude = current;

        G = acceleration.norm();
        float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
        gain = std::clamp(gain, 0.f, 1.f);

        Eigen::Quaternionf worldFrameAcc = (current.conjugate() *
                                            Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z())) *
                                           current;
        linearAcceleration = worldFrameAcc.vec() - Eigen::Vector3f{0, 0, 1};
        linearAcceleration *= 9.81;

        worldFrameAcc.coeffs() *= 1 / G;
        Eigen::Quaternionf accDeltaQ;
        accDeltaQ.w() = std::sqrt((worldFrameAcc.z() + 1) / 2);
        accDeltaQ.x() = -worldFrameAcc.y() / (2 * accDeltaQ.w());
        accDeltaQ.y() = worldFrameAcc.x() / (2 * accDeltaQ.w());
        accDeltaQ.z() = 0.0;
        accDeltaQ = adaptiveSLERP_I(accDeltaQ, gain);

        if (accDeltaQ.coeffs().allFinite())
            attitude = (current * accDeltaQ).normalized();

        update();
        correctAcc(linearAcceleration.z(),
                   accelerometerNoise * accelerometerNoise);
        predictKalman(dT);
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
    }

    void updateByPressure(float P)
    {
        pressure = P;
        correctPos(getAltitudeFromPressure(pressure, 101'325),
                   barometerNoise * barometerNoise);
    }

    void updateByTemperature(float T)
    {
        temperature = T;
    }

    Eigen::Vector3f getAcceleration() { return acceleration; }
    Eigen::Vector3f getRawAcceleration() { return rawAcceleration; }
    float getG() { return G; }

    Eigen::Vector3f getRotateAcceleration() { return Eigen::Vector3f(0, 0, 0); }

    Eigen::Vector3f getRawRotateSpeed() { return rawRotateSpeed; }
    Eigen::Vector3f getRotateSpeed() { return rotateSpeed; }
    Eigen::Vector3f getFRD_RotateSpeed() { return Eigen::Vector3f(rotateSpeed.x(), rotateSpeed.y(), -rotateSpeed.z()); }

    float getPressure() { return pressure; }
    float getTemperature() { return temperature; }

    Eigen::Quaternionf getFRU_Attitude() { return attitude; }
    Eigen::Quaternionf getFRD_Attitude() { return Eigen::Quaternionf(attitude.w(), attitude.x(), attitude.y(), -attitude.z()); }
    Eulerf getEulerFRU() { return eulerAttitude; }
    Eulerf getEulerFRD() { return Eulerf{eulerAttitude.roll, eulerAttitude.pitch, -eulerAttitude.yaw}; }

    Eigen::Vector3f getFRDLinearAcceleration() { return linearAcceleration; }

    float getLastDT() { return lastDt; }

    Eigen::Vector3f getZState() { return x; }
    Eigen::Vector3f getZVaraince() { return Eigen::Vector3f{P(0, 0), P(1, 1), P(2, 2)}; }
}
#pragma once

#include "Common.hpp"

struct MLData {
    uint8_t payload[4];
};

class MachineLearning {
private:
    float _dx = 0.0f;
    float _dy = 0.0f;
    float _dz = 0.0f;
    float _yaw = 0.0f;
public:
    void update(MLData &mlData);
    float getDx() { return _dx; }
    float getDy() { return _dy; }
    float getDz() { return _dz; }
    float getYaw() { return _yaw; }
};

MachineLearning ML;
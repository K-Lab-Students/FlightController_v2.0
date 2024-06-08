#include "MachineLearning.hpp"
#include "rc/RC.hpp"
#include "Board.hpp"

void MachineLearning::update(MLData &mlData) {
    _dx = byteToFloat(mlData.payload[0]);
    _dy = byteToFloat(mlData.payload[1]);
    _dz = byteToFloat(mlData.payload[2]);
    _yaw = byteToFloat(mlData.payload[3]);

    // debug
    if (false) {
        mav1Uart.print((int) mlData.payload[0], 10);
        mav1Uart.print('\t');
        mav1Uart.print((int) mlData.payload[1], 10);
        mav1Uart.print('\t');
        mav1Uart.print((int) mlData.payload[2], 10);
        mav1Uart.print('\t');
        mav1Uart.print((int) mlData.payload[3], 10);
        mav1Uart.print('\t');
        mav1Uart.print((int) (512 * RC::channel(RC::ChannelFunction::PARAM_1) + 512), 10);
        mav1Uart.print('\t');
        mav1Uart.print((int) c++, 10);
        mav1Uart.print('\n');
        mav1Uart.startTX();
    }
}

float MachineLearning::byteToFloat(uint8_t byte) {
    return 1.0 * byte / 255 - 0.5;
}

MachineLearning ML;

#include "Common.hpp"
#include "SRT/SRT.hpp"
#include "modes/Modes.hpp"
#include "Board.hpp"
#include "ml/MachineLearning.hpp"

const size_t buf_size = sizeof(MLData);
uint8_t buff[buf_size];

bool process_bytes(uint8_t* buf, size_t len) {
    static int state = 0;
    static size_t c = 0;
    while (mav1Uart.available() > 0) {
        int i;
        if ((i = mav1Uart.read()) < 0) {
            state = 0;
            return false;
        }
        uint8_t byte = i;
        switch (state) {
            case 0: {
                if (byte == 0xFF) {
                    state = 1;
                    break;
                }
            }
            case 1: {
                if (byte == 0xFF) {
                    state = 2;
                } else {
                    state = 0;
                    c = 0;
                }
                break;
            }
            default: {
                buf[c++] = byte;
                if (c == len) {
                    state = 0;
                    c = 0;
                    return true;
                } else {
                    return false;
                }
            }
        }
    }
    return false;
}


int main()
{
    setup();
    SRT::init();
    SRT::enable();

    mav1Uart.begin(115200);
    mav1Uart.setAutoSend(false);

    while (true)
    {
        loop();
        SRT::handler();
        FlightModeDispatcher::handler();
        if (process_bytes(buff, sizeof(MLData))) {
            ML.update(*(MLData*) buff);
        }
    }
}
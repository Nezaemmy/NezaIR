#include "IRremoteInt.h"



#define DISH_BITS             16
#define DISH_HEADER_MARK     400
#define DISH_HEADER_SPACE   6100
#define DISH_BIT_MARK        400
#define DISH_ONE_SPACE      1700
#define DISH_ZERO_SPACE     2800
#define DISH_REPEAT_SPACE   6200

//+=============================================================================
void IRsend::sendDISH(unsigned long data, int nbits) {
    // Set IR carrier frequency
    enableIROut(56);

    mark(DISH_HEADER_MARK);
    space(DISH_HEADER_SPACE);

    sendPulseDistanceWidthData(DISH_BIT_MARK, DISH_ONE_SPACE, DISH_BIT_MARK, DISH_ZERO_SPACE, data, nbits, PROTOCOL_IS_MSB_FIRST);
    mark(DISH_HEADER_MARK); //added 26th March 2016, by AnalysIR ( https://www.AnalysIR.com )
}


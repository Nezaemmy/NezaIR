#pragma once
#ifndef MULTI_IR_SHARED_H
#define MULTI_IR_SHARED_H
#include <Arduino.h>
#include "CppList.hpp"
CppList &IrReceiverRegistry();
extern volatile uint8_t gActiveReceivers;
#if defined(ESP32)
#include "esp32-hal-timer.h"
extern hw_timer_t *gIRReceiveTimer;
#endif
#endif

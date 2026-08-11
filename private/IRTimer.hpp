#pragma once
#ifndef IR_TIMER_HPP
#define IR_TIMER_HPP
#include <Arduino.h>
#include "IRremoteInt.h"
#include "../MultiIRShared.h"
#if !defined(ESP32)
#error "This NezaIR timer backend supports the Arduino-ESP32 family only."
#endif
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || (ESP_ARDUINO_VERSION_MAJOR < 3)
#error "NezaIR requires Arduino-ESP32 core 3.x or newer."
#endif
#ifndef IR_SEND_PIN
#define IR_SEND_PIN 4
#endif
#ifndef LED_CHANNEL
#define LED_CHANNEL 0
#endif
#define LEDC_DUTY_FROM_PERCENT(pct) ((uint32_t)(pct) * 255UL / 100UL)
#define ENABLE_SEND_PWM_BY_TIMER ledcWriteChannel(LED_CHANNEL, LEDC_DUTY_FROM_PERCENT(IR_SEND_DUTY_CYCLE))
#define DISABLE_SEND_PWM_BY_TIMER ledcWriteChannel(LED_CHANNEL, 0)
#define TIMER_RESET_INTR_PENDING do {} while (0)
ARDUINO_ISR_ATTR void IRTimerInterruptHandler();
static inline bool timerConfigForReceive() {
    if (gIRReceiveTimer) return true;
    gIRReceiveTimer = timerBegin(1000000UL);
    if (!gIRReceiveTimer) return false;
    timerAttachInterrupt(gIRReceiveTimer, &IRTimerInterruptHandler);
    timerAlarm(gIRReceiveTimer, (uint64_t)MICROS_PER_TICK, true, 0);
    timerStop(gIRReceiveTimer);
    return true;
}
static inline void timerEnableReceiveInterrupt() { if (gIRReceiveTimer) timerStart(gIRReceiveTimer); }
static inline void timerDisableReceiveInterrupt() {
    if (!gIRReceiveTimer) return;
    timerStop(gIRReceiveTimer);
    timerDetachInterrupt(gIRReceiveTimer);
    timerEnd(gIRReceiveTimer);
    gIRReceiveTimer=nullptr;
}
#define TIMER_ENABLE_RECEIVE_INTR timerEnableReceiveInterrupt()
#define TIMER_DISABLE_RECEIVE_INTR timerDisableReceiveInterrupt()
static inline void timerConfigForSend(uint8_t frequencyKHz) {
    ledcAttachChannel(IrSender.sendPin, (uint32_t)frequencyKHz * 1000UL, 8, LED_CHANNEL);
}
#endif

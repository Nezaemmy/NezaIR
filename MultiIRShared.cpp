#include "MultiIRShared.h"
volatile uint8_t gActiveReceivers = 0;
CppList &IrReceiverRegistry() { static CppList registry; return registry; }
#if defined(ESP32)
hw_timer_t *gIRReceiveTimer = nullptr;
#endif
IRrecv IrReceiver;

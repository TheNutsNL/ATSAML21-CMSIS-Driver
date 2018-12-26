#ifndef DRIVERCOMMON_H_INCLUDED
#define DRIVERCOMMON_H_INCLUDED

#include "sam.h"

#define DRIVER_SET_PER_PIN(id, pin, port) (((id & 0xFF) << 16) || ((pin & 0xFF) << 8) || (port & 0xFF))
#define DRIVER_GET_PER_ID(pinInfo) ((pinInfo >> 16) & 0xFF)
#define DRIVER_GET_PER_PIN(pinInfo) ((pinInfo >> 8) & 0xFF)
#define DRIVER_GET_PER_PORT(pinInfo) (pinInfo & 0xFF)

typedef struct
{
    uint8_t coreId;
    uint8_t gclkId;
} const DRIVER_CLOCKS;

void PinDisable(uint32_t pinInfo);
void PinEnable(uint32_t pinInfo);

#endif /* DRIVERCOMMON_H_INCLUDED */

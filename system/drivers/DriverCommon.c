#include "DriverCommon.h"

void PinDisable(uint32_t pinInfo)
{
    PORT->Group[DRIVER_GET_PER_PORT(pinInfo)].PINCFG[DRIVER_GET_PER_ID(pinInfo)].reg = PORT_PINCFG_RESETVALUE;
}

void PinEnable(uint32_t pinInfo)
{
    PORT->Group[DRIVER_GET_PER_PORT(pinInfo)].PINCFG[DRIVER_GET_PER_PIN(pinInfo)].reg = PORT_PINCFG_PMUXEN;
    if (DRIVER_GET_PER_PIN(pinInfo) & 0x1)
        PORT->Group[DRIVER_GET_PER_PORT(pinInfo)].PMUX[DRIVER_GET_PER_PIN(pinInfo) >> 1].bit.PMUXO = DRIVER_GET_PER_ID(pinInfo);
    else
        PORT->Group[DRIVER_GET_PER_PORT(pinInfo)].PMUX[DRIVER_GET_PER_PIN(pinInfo) >> 1].bit.PMUXE = DRIVER_GET_PER_ID(pinInfo);
}



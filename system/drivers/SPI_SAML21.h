#ifndef SPI_SAML21_H_INCLUDED
#define SPI_SAML21_H_INCLUDED

#include "DriverCommon.h"
#include "Driver_SPI.h"
#include "SERCOM_SAML21.h"

//SPI flags
#define SPI_FLAG_INITIALIZED        0x01
#define SPI_FLAG_POWERED            0x02
#define SPI_FLAG_ENABLED            0x04
#define SPI_FLAG_SS_SOFTWARE        0x40
#define SPI_FLAG_SS_HARDWARE        0x80

typedef struct
{
    ARM_SPI_SignalEvent_t cb_event;
    ARM_SPI_STATUS status;
    uint32_t bytesToTransfer;
    uint32_t bytesSent, bytesRecieved;
    const uint8_t *dataOut;
    uint8_t *dataIn;
    uint8_t flags;
    uint8_t defaultTX;
} SPI_INFO;

typedef struct
{
    SERCOM_Resource *sercom_res;
    uint8_t DOPO;
    uint8_t DIPO;
    SPI_INFO *info;
} const SPI_RESOURCE;

static int32_t SPIx_PowerControl(const SPI_RESOURCE *res, ARM_POWER_STATE state);
static int32_t SPIx_Control(const SPI_RESOURCE *res, uint32_t control, uint32_t arg);

#endif /* SPI_SAML21_H_INCLUDED */

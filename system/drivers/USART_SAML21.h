#ifndef USART_SAML21_H_INCLUDED
#define USART_SAML21_H_INCLUDED

#include "DriverCommon.h"
#include "Driver_USART.h"
#include "SERCOM_SAML21.h"

//USART flags
#define USART_FLAG_INITIALIZED          0x01
#define USART_FLAG_POWERED              0x02
#define USART_FLAG_ENABLED              0x04
#define USART_FLAG_MODE_ASYNCHRONOUS    0x08
#define USART_FLAG_MODE_SYNCHRONOUS     0x10

typedef struct
{
    ARM_USART_SignalEvent_t cb_event;
    ARM_USART_STATUS status;
    uint32_t rxCount, txCount;
    const uint8_t *dataOut;
    uint8_t *dataIn;
    uint32_t flags;
} USART_INFO;

typedef struct
{
    SERCOM_Resource* sercom_res;
    uint32_t RXPO;
    uint32_t TXPO;
    USART_INFO* info;
}const USART_RESOURCE;

#endif /* USART_SAML21_H_INCLUDED */

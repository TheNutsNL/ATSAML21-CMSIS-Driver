#ifndef SERCOM_SAML21_H_INCLUDED
#define SERCOM_SAML21_H_INCLUDED

#include "sam.h"

typedef struct
{
    Sercom* sercom;
    uint32_t pad[4];
    IRQn_Type irq;
    uint8_t coreId;
    uint8_t gclkId;
}const SERCOM_Resource;

void Sercom_PeriphalClockDisable(const SERCOM_Resource* res);
void Sercom_PeriphalClockEnable(const SERCOM_Resource* res);

extern SERCOM_Resource sercom0_res;
extern SERCOM_Resource sercom1_res;
extern SERCOM_Resource sercom2_res;
extern SERCOM_Resource sercom3_res;
extern SERCOM_Resource sercom4_res;
extern SERCOM_Resource sercom5_res;

#endif /* SERCOM_SAML21_H_INCLUDED */

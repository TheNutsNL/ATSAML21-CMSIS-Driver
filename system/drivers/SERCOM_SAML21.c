#include "SERCOM_SAML21.h"
#include "RTE_Device.h"
#include "DriverCommon.h"

void Sercom_PeriphalClockDisable(const SERCOM_Resource* res)
{
    GCLK->PCHCTRL[res->coreId].bit.CHEN = 0;
    while (GCLK->PCHCTRL[res->coreId].bit.CHEN);
}

void Sercom_PeriphalClockEnable(const SERCOM_Resource* res)
{
    //Enable peripheral clock
    GCLK->PCHCTRL[res->coreId].bit.CHEN = 0;
    while (GCLK->PCHCTRL[res->coreId].bit.CHEN);

    GCLK->PCHCTRL[res->coreId].bit.GEN = res->gclkId;
    GCLK->PCHCTRL[res->coreId].bit.CHEN = 1;
}

#ifndef RTE_SERCOM0_MODE
    #define RTE_SERCOM0_MODE RTE_SERCOM_MODE_NONE
#endif

#ifndef RTE_SERCOM1_MODE
    #define RTE_SERCOM1_MODE RTE_SERCOM_MODE_NONE
#endif

#ifndef RTE_SERCOM2_MODE
    #define RTE_SERCOM2_MODE RTE_SERCOM_MODE_NONE
#endif

#ifndef RTE_SERCOM3_MODE
    #define RTE_SERCOM3_MODE RTE_SERCOM_MODE_NONE
#endif

#ifndef RTE_SERCOM4_MODE
    #define RTE_SERCOM4_MODE RTE_SERCOM_MODE_NONE
#endif

#ifndef RTE_SERCOM5_MODE
    #define RTE_SERCOM5_MODE RTE_SERCOM_MODE_NONE
#endif

//Define SERCOM0
#if (RTE_SERCOM0_MODE != RTE_SERCOM_MODE_NONE)
    //Define pads
    #ifndef RTE_SERCOM0_PAD0_ALT
        #define SERCOM0_PAD0 (DRIVER_SET_PER_PIN(2, PIN_PA08, 0))
    #else
        #define SERCOM0_PAD0 (DRIVER_SET_PER_PIN(3, PIN_PA04, 0))
    #endif

    #ifndef RTE_SERCOM0_PAD1_ALT
        #define SERCOM0_PAD1 (DRIVER_SET_PER_PIN(2, PIN_PA09, 0))
    #else
        #define SERCOM0_PAD1 (DRIVER_SET_PER_PIN(3, PIN_PA05, 0))
    #endif

    #ifndef RTE_SERCOM0_PAD2_ALT
        #define SERCOM0_PAD2 (DRIVER_SET_PER_PIN(2, PIN_PA10, 0))
    #else
        #define SERCOM0_PAD2 (DRIVER_SET_PER_PIN(3, PIN_PA06, 0))
    #endif

    #ifndef RTE_SERCOM0_PAD3_ALT
        #define SERCOM0_PAD3 (DRIVER_SET_PER_PIN(2, PIN_PA11, 0))
    #else
        #define SERCOM0_PAD3 (DRIVER_SET_PER_PIN(3, PIN_PA07, 0))
    #endif

    SERCOM_Resource sercom0_res =
    {
        SERCOM0,
        {
            SERCOM0_PAD0,
            SERCOM0_PAD1,
            SERCOM0_PAD2,
            SERCOM0_PAD3
        },
        SERCOM0_IRQn,
        SERCOM0_GCLK_ID_CORE,
        RTE_SERCOM0_GCLK
    };
#endif

//Define SERCOM1
#if (RTE_SERCOM1_MODE != RTE_SERCOM_MODE_NONE)
    //Define pads
    #ifndef RTE_SERCOM1_PAD0_ALT
        #define SERCOM1_PAD0 (DRIVER_SET_PER_PIN(2, PIN_PA16, 0))
    #else
        #define SERCOM1_PAD0 (DRIVER_SET_PER_PIN(3, PIN_PA00, 0))
    #endif

    #ifndef RTE_SERCOM1_PAD1_ALT
        #define SERCOM1_PAD1 (DRIVER_SET_PER_PIN(2, PIN_PA17, 0))
    #else
        #define SERCOM1_PAD1 (DRIVER_SET_PER_PIN(3, PIN_PA01, 0))
    #endif

    #ifndef RTE_SERCOM1_PAD2_ALT
        #define SERCOM1_PAD2 (DRIVER_SET_PER_PIN(2, PIN_PA18, 0))
    #else
        #define SERCOM1_PAD2 (DRIVER_SET_PER_PIN(3, PIN_PA30, 0))
    #endif

    #ifndef RTE_SERCOM1_PAD3_ALT
        #define SERCOM1_PAD3 (DRIVER_SET_PER_PIN(2, PIN_PA19, 0))
    #else
        #define SERCOM1_PAD3 (DRIVER_SET_PER_PIN(3, PIN_PA31, 0))
    #endif

    SERCOM_Resource sercom1_res =
    {
        SERCOM1,
        {
            SERCOM1_PAD0,
            SERCOM1_PAD1,
            SERCOM1_PAD2,
            SERCOM1_PAD3
        },
        SERCOM1_IRQn,
        SERCOM1_GCLK_ID_CORE,
        RTE_SERCOM1_GCLK
    };
#endif

//Define SERCOM2
#if (RTE_SERCOM2_MODE != RTE_SERCOM_MODE_NONE)
    //Define pads
    #ifndef RTE_SERCOM2_PAD0_ALT
        #define SERCOM2_PAD0 (DRIVER_SET_PER_PIN(2, PIN_PA12, 0))
    #else
        #define SERCOM2_PAD0 (DRIVER_SET_PER_PIN(3, PIN_PA08, 0))
    #endif

    #ifndef RTE_SERCOM2_PAD1_ALT
        #define SERCOM2_PAD1 (DRIVER_SET_PER_PIN(2, PIN_PA13, 0))
    #else
        #define SERCOM2_PAD1 (DRIVER_SET_PER_PIN(3, PIN_PA09, 0))
    #endif

    #ifndef RTE_SERCOM2_PAD2_ALT
        #define SERCOM2_PAD2 (DRIVER_SET_PER_PIN(2, PIN_PA14, 0))
    #else
        #define SERCOM2_PAD2 (DRIVER_SET_PER_PIN(3, PIN_PA10, 0))
    #endif

    #ifndef RTE_SERCOM2_PAD3_ALT
        #define SERCOM2_PAD3 (DRIVER_SET_PER_PIN(2, PIN_PA15, 0))
    #else
        #define SERCOM2_PAD3 (DRIVER_SET_PER_PIN(3, PIN_PA11, 0))
    #endif

    SERCOM_Resource sercom2_res =
    {
        SERCOM2,
        {
            SERCOM2_PAD0,
            SERCOM2_PAD1,
            SERCOM2_PAD2,
            SERCOM2_PAD3
        },
        SERCOM2_IRQn,
        SERCOM2_GCLK_ID_CORE,
        RTE_SERCOM2_GCLK
    };
#endif

//Define SERCOM3
#if (RTE_SERCOM3_MODE != RTE_SERCOM_MODE_NONE)
    //Define pads
    #ifndef RTE_SERCOM3_PAD0_ALT
        #define SERCOM3_PAD0 (DRIVER_SET_PER_PIN(2, PIN_PA22, 0))
    #else
        #define SERCOM3_PAD0 (DRIVER_SET_PER_PIN(3, PIN_PA16, 0))
    #endif

    #ifndef RTE_SERCOM3_PAD1_ALT
        #define SERCOM3_PAD1 (DRIVER_SET_PER_PIN(2, PIN_PA23, 0))
    #else
        #define SERCOM3_PAD1 (DRIVER_SET_PER_PIN(3, PIN_PA17, 0))
    #endif

    #ifndef RTE_SERCOM3_PAD2_ALT
        #define SERCOM3_PAD2 (DRIVER_SET_PER_PIN(2, PIN_PA24, 0))
    #else
        #define SERCOM3_PAD2 (DRIVER_SET_PER_PIN(3, PIN_PA18, 0))
    #endif

    #ifndef RTE_SERCOM3_PAD3_ALT
        #define SERCOM3_PAD3 (DRIVER_SET_PER_PIN(2, PIN_PA25, 0))
    #else
        #define SERCOM3_PAD3 (DRIVER_SET_PER_PIN(3, PIN_PA19, 0))
    #endif

    SERCOM_Resource sercom3_res =
    {
        SERCOM3,
        {
            SERCOM3_PAD0,
            SERCOM3_PAD1,
            SERCOM3_PAD2,
            SERCOM3_PAD3
        },
        SERCOM3_IRQn,
        SERCOM3_GCLK_ID_CORE,
        RTE_SERCOM3_GCLK
    };
#endif

//Define SERCOM4
#if (RTE_SERCOM4_MODE != RTE_SERCOM_MODE_NONE)
    //Define pads
    #ifndef RTE_SERCOM4_PAD0_ALT
        #define SERCOM4_PAD0 (DRIVER_SET_PER_PIN(2, PIN_PB12, 1))
    #else
        #define SERCOM4_PAD0 (DRIVER_SET_PER_PIN(3, PIN_PA12, 0))
    #endif

    #ifndef RTE_SERCOM4_PAD1_ALT
        #define SERCOM4_PAD1 (DRIVER_SET_PER_PIN(2, PIN_PB13, 1))
    #else
        #define SERCOM4_PAD1 (DRIVER_SET_PER_PIN(3, PIN_PA13, 0))
    #endif

    #ifndef RTE_SERCOM4_PAD2_ALT
        #define SERCOM4_PAD2 (DRIVER_SET_PER_PIN(2, PIN_PB14, 1))
    #else
        #define SERCOM4_PAD2 (DRIVER_SET_PER_PIN(3, PIN_PA14, 0))
    #endif

    #ifndef RTE_SERCOM4_PAD3_ALT
        #define SERCOM4_PAD3 (DRIVER_SET_PER_PIN(2, PIN_PB15, 1))
    #else
        #define SERCOM4_PAD3 (DRIVER_SET_PER_PIN(3, PIN_PA15, 0))
    #endif

    SERCOM_Resource sercom4_res =
    {
        SERCOM4,
        {
            SERCOM4_PAD0,
            SERCOM4_PAD1,
            SERCOM4_PAD2,
            SERCOM4_PAD3
        },
        SERCOM4_IRQn,
        SERCOM4_GCLK_ID_CORE,
        RTE_SERCOM4_GCLK
    };
#endif

//Define SERCOM5
#if (RTE_SERCOM5_MODE != RTE_SERCOM_MODE_NONE)
    //Define pads
    #ifndef RTE_SERCOM5_PAD0_ALT
        #define SERCOM5_PAD0 (DRIVER_SET_PER_PIN(2, PIN_PB16, 1))
    #else
        #define SERCOM5_PAD0 (DRIVER_SET_PER_PIN(3, PIN_PA22, 0))
    #endif

    #ifndef RTE_SERCOM5_PAD1_ALT
        #define SERCOM5_PAD1 (DRIVER_SET_PER_PIN(2, PIN_PB17, 1))
    #else
        #define SERCOM5_PAD1 (DRIVER_SET_PER_PIN(3, PIN_PA23, 0))
    #endif

    #ifndef RTE_SERCOM5_PAD2_ALT
        #define SERCOM5_PAD2 (DRIVER_SET_PER_PIN(2, PIN_PA20, 0))
    #else
        #define SERCOM5_PAD2 (DRIVER_SET_PER_PIN(3, PIN_PA24, 0))
    #endif

    #ifndef RTE_SERCOM5_PAD3_ALT
        #define SERCOM5_PAD3 (DRIVER_SET_PER_PIN(2, PIN_PA21, 0))
    #else
        #define SERCOM5_PAD3 (DRIVER_SET_PER_PIN(3, PIN_PA25, 0))
    #endif

    SERCOM_Resource sercom5_res =
    {
        SERCOM5,
        {
            SERCOM5_PAD0,
            SERCOM5_PAD1,
            SERCOM5_PAD2,
            SERCOM5_PAD3
        },
        SERCOM5_IRQn,
        SERCOM5_GCLK_ID_CORE,
        RTE_SERCOM5_GCLK
    };
#endif

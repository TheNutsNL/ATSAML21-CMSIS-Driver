#ifndef RTE_DEVICE_H_INCLUDED
#define RTE_DEVICE_H_INCLUDED

#define RTE_SERCOM_MODE_NONE    0
#define RTE_SERCOM_MODE_USART   1
#define RTE_SERCOM_MODE_SPI     2
#define RTE_SERCOM_MODE_I2C     3

//Sercom setup
#define RTE_SERCOM0_MODE  RTE_SERCOM_MODE_SPI
#define RTE_SERCOM1_MODE  RTE_SERCOM_MODE_USART
#define RTE_SERCOM2_MODE  RTE_SERCOM_MODE_NONE
#define RTE_SERCOM3_MODE  RTE_SERCOM_MODE_NONE
#define RTE_SERCOM4_MODE  RTE_SERCOM_MODE_NONE
#define RTE_SERCOM5_MODE  RTE_SERCOM_MODE_NONE
#define RTE_SERCOM6_MODE  RTE_SERCOM_MODE_NONE

#if (RTE_SERCOM0_MODE != RTE_SERCOM_MODE_NONE)
    #define RTE_SERCOM0_GCLK 0      //Generic clock source

    //For alternate pad define RTE_SERCOM0_PADx_ALT
    //#define RTE_SERCOM0_PAD0_ALT
    //#define RTE_SERCOM0_PAD1_ALT
    //#define RTE_SERCOM0_PAD2_ALT
    //#define RTE_SERCOM0_PAD3_ALT

    #if (RTE_SERCOM0_MODE == RTE_SERCOM_MODE_USART)
        #define RTE_USART0_RXPO     1
        #define RTE_USART0_TXPO     2
    #elif (RTE_SERCOM0_MODE == RTE_SERCOM_MODE_SPI)
        #define RTE_SPI0_DOPO       0
        #define RTE_SPI0_DIPO       3
    #elif (RTE_SERCOM0_MODE == RTE_SERCOM_MODE_I2C)
    #else
        #error "Invalid value for RTE_SERCOM0_MODE!"
    #endif
#endif

#if (RTE_SERCOM1_MODE != RTE_SERCOM_MODE_NONE)
    #define RTE_SERCOM1_GCLK 0      //Generic clock source

    //For alternate pad define RTE_SERCOM1_PADx_ALT
    //#define RTE_SERCOM1_PAD0_ALT
    //#define RTE_SERCOM1_PAD1_ALT
    //#define RTE_SERCOM1_PAD2_ALT
    //#define RTE_SERCOM1_PAD3_ALT

    #if (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_USART)
        #define RTE_USART1_RXPO     1
        #define RTE_USART1_TXPO     2
    #elif (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_SPI)
        #define RTE_SPI1_DOPO       0
        #define RTE_SPI1_DIPO       3
    #elif (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_I2C)

    #endif
#endif

#if (RTE_SERCOM2_MODE != RTE_SERCOM_MODE_NONE)
    #define RTE_SERCOM2_GCLK 0      //Generic clock source

    //For alternate pad define RTE_SERCOM2_PADx_ALT
    //#define RTE_SERCOM2_PAD0_ALT
    //#define RTE_SERCOM2_PAD1_ALT
    //#define RTE_SERCOM2_PAD2_ALT
    //#define RTE_SERCOM2_PAD3_ALT

    #if (RTE_SERCOM2_MODE == RTE_SERCOM_MODE_USART)
        #define RTE_USART2_RXPO     1
        #define RTE_USART2_TXPO     2
    #elif (RTE_SERCOM2_MODE == RTE_SERCOM_MODE_SPI)
        #define RTE_SPI2_DOPO       0
        #define RTE_SPI2_DIPO       3
    #elif (RTE_SERCOM2_MODE == RTE_SERCOM_MODE_I2C)

    #endif
#endif

#if (RTE_SERCOM3_MODE != RTE_SERCOM_MODE_NONE)
    #define RTE_SERCOM3_GCLK 0      //Generic clock source

    //For alternate pad define RTE_SERCOM3_PADx_ALT
    //#define RTE_SERCOM3_PAD0_ALT
    //#define RTE_SERCOM3_PAD1_ALT
    //#define RTE_SERCOM3_PAD2_ALT
    //#define RTE_SERCOM3_PAD3_ALT

    #if (RTE_SERCOM3_MODE == RTE_SERCOM_MODE_USART)
        #define RTE_USART3_RXPO     1
        #define RTE_USART3_TXPO     2
    #elif (RTE_SERCOM3_MODE == RTE_SERCOM_MODE_SPI)
        #define RTE_SPI3_DOPO       0
        #define RTE_SPI3_DIPO       3
    #elif (RTE_SERCOM3_MODE == RTE_SERCOM_MODE_I2C)

    #endif
#endif

#if (RTE_SERCOM4_MODE != RTE_SERCOM_MODE_NONE)
    #define RTE_SERCOM4_GCLK 0      //Generic clock source

    //For alternate pad define RTE_SERCOM4_PADx_ALT
    //#define RTE_SERCOM4_PAD0_ALT
    //#define RTE_SERCOM4_PAD1_ALT
    //#define RTE_SERCOM4_PAD2_ALT
    //#define RTE_SERCOM4_PAD3_ALT

    #if (RTE_SERCOM4_MODE == RTE_SERCOM_MODE_USART)
        #define RTE_USART4_RXPO     1
        #define RTE_USART4_TXPO     2
    #elif (RTE_SERCOM4_MODE == RTE_SERCOM_MODE_SPI)
        #define RTE_SPI4_DOPO       0
        #define RTE_SPI4_DIPO       3
    #elif (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_I2C)

    #endif
#endif

#if (RTE_SERCOM5_MODE != RTE_SERCOM_MODE_NONE)
    #define RTE_SERCOM5_GCLK 0      //Generic clock source

    //For alternate pad define RTE_SERCOM5_PADx_ALT
    //#define RTE_SERCOM5_PAD0_ALT
    //#define RTE_SERCOM5_PAD1_ALT
    //#define RTE_SERCOM5_PAD2_ALT
    //#define RTE_SERCOM5_PAD3_ALT

    #if (RTE_SERCOM5_MODE == RTE_SERCOM_MODE_USART)
        #define RTE_USART5_RXPO     1
        #define RTE_USART5_TXPO     2
    #elif (RTE_SERCOM5_MODE == RTE_SERCOM_MODE_SPI)
        #define RTE_SPI5_DOPO       0
        #define RTE_SPI5_DIPO       3
    #elif (RTE_SERCOM5_MODE == RTE_SERCOM_MODE_I2C)

    #endif
#endif

#endif /* RTE_DEVICE_H_INCLUDED */

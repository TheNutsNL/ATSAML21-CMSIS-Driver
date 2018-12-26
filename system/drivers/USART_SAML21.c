#include "USART_SAML21.h"
#include "RTE_Device.h"
#include "SERCOM_SAML21.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0)  /* driver version */

static int32_t USARTx_PowerControl(const USART_RESOURCE *res, ARM_POWER_STATE state);

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

//
//   Functions
//

static ARM_DRIVER_VERSION USART_GetVersion(void)
{

    return DriverVersion;
}

static ARM_USART_CAPABILITIES USARTx_GetCapabilities(const USART_RESOURCE *res)
{
    ARM_USART_CAPABILITIES driverCapabilities=
    {
        1, /* supports UART (Asynchronous) mode */
        0, /* supports Synchronous Master mode */
        0, /* supports Synchronous Slave mode */
        0, /* supports UART Single-wire mode */
        1, /* supports UART IrDA mode */
        0, /* supports UART Smart Card mode */
        0, /* Smart Card Clock generator available */
        0, /* RTS Flow Control available */
        0, /* CTS Flow Control available */
        1, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
        0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
        0, /* RTS Line: 0=not available, 1=available */
        0, /* CTS Line: 0=not available, 1=available */
        0, /* DTR Line: 0=not available, 1=available */
        0, /* DSR Line: 0=not available, 1=available */
        0, /* DCD Line: 0=not available, 1=available */
        0, /* RI Line: 0=not available, 1=available */
        0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
        0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
        0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
        0  /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    };

    switch (res->TXPO)
    {
    case 0:
    case 1:
        driverCapabilities.synchronous_master = 1;
        driverCapabilities.synchronous_slave = 1;
        break;

    case 2:
        driverCapabilities.flow_control_rts = 1;
        driverCapabilities.flow_control_cts = 1;
        driverCapabilities.rts = 1;
        driverCapabilities.cts = 1;
        driverCapabilities.event_cts = 1;
        break;
    }

    return driverCapabilities;
}

static int32_t USARTx_Initialize(const USART_RESOURCE *res, ARM_USART_SignalEvent_t cb_event)
{
    if (res->info->flags & USART_FLAG_INITIALIZED)
        return ARM_DRIVER_OK;

    //Resister call back event
    res->info->cb_event = cb_event;
    //TO DO: reset status


    //Enable pins
    PinEnable(res->sercom_res->pad[res->RXPO]);
    switch (res->TXPO)
    {
    case 0:
    case 2:
        PinEnable(res->sercom_res->pad[0]);
        break;
    case 1:
        PinEnable(res->sercom_res->pad[2]);
        break;
    }

    //Flag initiated flag
    res->info->flags = USART_FLAG_INITIALIZED;

    return ARM_DRIVER_OK;
}

static int32_t USARTx_Uninitialize(const USART_RESOURCE *res)
{
    if (res->info->flags & USART_FLAG_POWERED)
        USARTx_PowerControl(res, ARM_POWER_OFF);

    //Disable pins
    PinDisable(res->sercom_res->pad[res->RXPO]);
    switch (res->TXPO)
    {
    case 0:
    case 2:
        PinDisable(res->sercom_res->pad[0]);
        break;
    case 1:
        PinDisable(res->sercom_res->pad[2]);
        break;
    }

    //Clear flags
    res->info->flags = 0;

    return ARM_DRIVER_OK;
}

static int32_t USARTx_PowerControl(const USART_RESOURCE *res, ARM_POWER_STATE state)
{
    switch (state)
    {
    case ARM_POWER_OFF:
        //Disable interrupt
        NVIC_DisableIRQ(res->sercom_res->irq);

        //Disable peripheral clock
        Sercom_PeriphalClockDisable(res->sercom_res);

        //TO DO:
        //Reset status
        //res->info->status.rx_busy = 0;
        //res->info->status.tx_busy = 0;

        //Clear powered flag
        res->info->flags &= ~USART_FLAG_POWERED;

        break;

//    case ARM_POWER_LOW:
//        break;

    case ARM_POWER_FULL:
        if ((res->info->flags & USART_FLAG_INITIALIZED )== 0)
            return ARM_DRIVER_ERROR;

        if (res->info->flags & USART_FLAG_POWERED)
            return ARM_DRIVER_OK;

        //Enable peripheral clock
        Sercom_PeriphalClockEnable(res->sercom_res);

        //Clear and enable interrupt
        NVIC_ClearPendingIRQ(res->sercom_res->irq);
        NVIC_EnableIRQ(res->sercom_res->irq);

        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static int32_t USARTx_Send(const USART_RESOURCE *res, const void *data, uint32_t num)
{
    return ARM_DRIVER_OK;
}

static int32_t USARTx_Receive(const USART_RESOURCE *res, void *data, uint32_t num)
{
    return ARM_DRIVER_OK;
}

static int32_t USARTx_Transfer(const USART_RESOURCE *res, const void *data_out, void *data_in, uint32_t num)
{
    return ARM_DRIVER_OK;
}

static int32_t USARTx_Control(const USART_RESOURCE *res, uint32_t control, uint32_t arg)
{
    SercomUsart *const usart = &res->sercom_res->sercom->USART;
    uint32_t ctrla = 0;
    uint32_t ctrlb;

    switch (control & ARM_USART_CONTROL_Msk)
    {
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_USART_MODE_ASYNCHRONOUS:
        ctrla = SERCOM_USART_CTRLA_MODE(1);
        break;

    case ARM_USART_MODE_SYNCHRONOUS_MASTER:
        if (res->TXPO > 1)
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        ctrla = SERCOM_USART_CTRLA_CMODE | SERCOM_USART_CTRLA_MODE(1);
        break;

    case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
        if (res->TXPO > 1)
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        ctrla = SERCOM_USART_CTRLA_CMODE;
        break;
    }

    if (ctrla)
    {
        if (res->info->flags & ~USART_FLAG_POWERED)
            return ARM_DRIVER_ERROR;

        //Setup pads
        ctrla |= SERCOM_USART_CTRLA_RXPO(res->RXPO) | SERCOM_USART_CTRLA_TXPO(res->TXPO);

        //Set data bits
        switch (control & ARM_USART_DATA_BITS_Msk)
        {
        case ARM_USART_DATA_BITS_5:
            ctrlb = SERCOM_USART_CTRLB_CHSIZE(5);
            break;

        case ARM_USART_DATA_BITS_6:
            ctrlb = SERCOM_USART_CTRLB_CHSIZE(6);
            break;

        case ARM_USART_DATA_BITS_7:
            ctrlb = SERCOM_USART_CTRLB_CHSIZE(7);
            break;

        case ARM_USART_DATA_BITS_8:
            ctrlb = SERCOM_USART_CTRLB_CHSIZE(0);
            break;

        case ARM_USART_DATA_BITS_9:
            ctrlb = SERCOM_USART_CTRLB_CHSIZE(1);
            break;

        default:
            return ARM_DRIVER_ERROR;
        }

        //Set parity
        switch (control & ARM_USART_PARITY_Msk)
        {
        case ARM_USART_PARITY_NONE:
            break;
        case ARM_USART_PARITY_ODD:
            ctrlb |= SERCOM_USART_CTRLB_PMODE;

        case ARM_USART_PARITY_EVEN:
            ctrla |= SERCOM_USART_CTRLA_FORM(1);
            break;

        }

        //Set stop bits
        switch (control & ARM_USART_STOP_BITS_Msk)
        {
        case ARM_USART_STOP_BITS_1:
            break;

        case ARM_USART_STOP_BITS_2:
            ctrlb |= SERCOM_USART_CTRLB_SBMODE;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        //Set clock polarity
        if ((control & ARM_USART_CPOL_Msk) == ARM_USART_CPOL1)
            ctrla |= SERCOM_USART_CTRLA_CPOL;

        //Set clock phase
        //if ((control & ARM_USART_CPHA_Msk) == ARM_USART_CPHA1)
        //    ctrla |= SERCOM_USART_CTRLB_CPHA;

        //Reset SERCOM
        usart->CTRLA.bit.SWRST = 1;
        while (usart->SYNCBUSY.bit.SWRST);

        usart->CTRLA.reg = ctrla;
        usart->CTRLB.reg = ctrlb;

    }

    return ARM_DRIVER_OK;
}

static int32_t USARTx_SetModemControl(const USART_RESOURCE *res, ARM_USART_MODEM_CONTROL control)
{
    return ARM_DRIVER_OK;
}

static ARM_USART_MODEM_STATUS USARTx_GetModemStatus(const USART_RESOURCE *res)
{
    //return ARM_DRIVER_OK;
}

void USARTx_Handler(const USART_RESOURCE *res)
{
}

//Configure USART1
#if (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_USART)
static USART_INFO USART1_info = {0, {0}, 0, 0, 0, 0, 0, 0};
static const USART_RESOURCE USART1_res =
{
    &sercom1_res,
    RTE_USART1_RXPO,
    RTE_USART1_TXPO,
    &USART1_info,
};

void SERCOM1_Handler() { USARTx_Handler(&USART1_res);}
static ARM_USART_CAPABILITIES USART1_GetCapabilities()                              { return USARTx_GetCapabilities(&USART1_res); }
static int32_t USART1_Initialize(ARM_USART_SignalEvent_t cb_event)                  { return USARTx_Initialize(&USART1_res, cb_event); }
static int32_t USART1_Uninitialize()                                                { return USARTx_Uninitialize(&USART1_res); }
static int32_t USART1_PowerControl(ARM_POWER_STATE state)                           { return USARTx_PowerControl(&USART1_res, state); }
static int32_t USART1_Send(const void *data, uint32_t num)                          { return USARTx_Send(&USART1_res, data, num); }
static int32_t USART1_Receive(void *data, uint32_t num)                             { return USARTx_Receive(&USART1_res, data, num); }
static int32_t USART1_Transfer(const void *data_out, void *data_in, uint32_t num)   { return USARTx_Transfer(&USART1_res, data_out, data_in, num); }
static uint32_t USART1_GetTxCount()                                                 { return USART1_res.info->txCount;}
static uint32_t USART1_GetRxCount()                                                 { return USART1_res.info->rxCount;}
static int32_t USART1_Control(uint32_t control, uint32_t arg)                       { return USARTx_Control(&USART1_res, control, arg); }
static ARM_USART_STATUS USART1_GetStatus()                                          { return USART1_res.info->status; }
static int32_t USART1_SetModemControl(ARM_USART_MODEM_CONTROL control)              { return USARTx_SetModemControl(&USART1_res, control); }
static ARM_USART_MODEM_STATUS USART1_GetModemStatus()                               { return USARTx_GetModemStatus(&USART1_res); }

ARM_DRIVER_USART Driver_USART1 =
{
    USART_GetVersion,
    USART1_GetCapabilities,
    USART1_Initialize,
    USART1_Uninitialize,
    USART1_PowerControl,
    USART1_Send,
    USART1_Receive,
    USART1_Transfer,
    USART1_GetTxCount,
    USART1_GetRxCount,
    USART1_Control,
    USART1_GetStatus,
    USART1_SetModemControl,
    USART1_GetModemStatus
};

#endif

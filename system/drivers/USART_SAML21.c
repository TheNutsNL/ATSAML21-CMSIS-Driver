#include "RTE_Device.h"
#include "SERCOM_SAML21.h"
#include "USART_SAML21.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0)  /* driver version */

static int32_t USARTx_PowerControl(const USART_RESOURCE *res, ARM_POWER_STATE state);
static void USART_ResetStatus(USART_INFO *info);
static int32_t USARTx_Transfer_Synchronous(const USART_RESOURCE *res, const void *dataOut, void *dataIn, uint32_t num);

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

//
//   Functions
//

void USART_ResetStatus(USART_INFO *info)
{
    info->flags = 0;
    info->bytesToSend = 0;
    info->bytesReceived = 0;
    info->bytesSent = 0;

    *((uint32_t*) &info->status) = 0;
}


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
    USART_ResetStatus(res->info);

    //Enable RxD and TxD pins
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

    //Disable RxD and TxD pins
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
        //Disable erro interrupt
        res->sercom_res->sercom->USART.INTENCLR.bit.ERROR = 1;

        //Disable interrupts
        NVIC_DisableIRQ(res->sercom_res->irq);

        //Disable peripheral clock
        Sercom_PeriphalClockDisable(res->sercom_res);

        USART_ResetStatus(res->info);

        //Clear powered flag
        res->info->flags &= ~USART_FLAG_POWERED;

        break;

//    case ARM_POWER_LOW:
//        break;

    case ARM_POWER_FULL:
        //Check if USART has been initialized
        if ((res->info->flags & USART_FLAG_INITIALIZED )== 0)
            return ARM_DRIVER_ERROR;

        //No action required if USART already powered
        if (res->info->flags & USART_FLAG_POWERED)
            return ARM_DRIVER_OK;

        //Reset registers
        res->sercom_res->sercom->USART.CTRLA.bit.SWRST = 1;
        while (res->sercom_res->sercom->USART.SYNCBUSY.bit.SWRST);

        //Enable peripheral clock
        Sercom_PeriphalClockEnable(res->sercom_res);

        //Clear and enable interrupt
        NVIC_ClearPendingIRQ(res->sercom_res->irq);
        NVIC_EnableIRQ(res->sercom_res->irq);

        //Enable error interrupt
        res->sercom_res->sercom->USART.INTENSET.bit.ERROR = 1;

        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}


static int32_t USARTx_Control(const USART_RESOURCE *res, uint32_t control, uint32_t arg)
{
    SercomUsart *usart = &res->sercom_res->sercom->USART;
    uint32_t ctrla = 0;
    uint32_t ctrlb;
    uint16_t baud;
    uint32_t temp;

    switch (control & ARM_USART_CONTROL_Msk)
    {
    case ARM_USART_MODE_IRDA:
    case ARM_USART_MODE_ASYNCHRONOUS:
        //Asynchronous mode using internal clock
        ctrla = SERCOM_USART_CTRLA_MODE(1);

        if ((control & ARM_USART_CONTROL_Msk) == ARM_USART_MODE_IRDA)
            //Enable IrDA encoding
            ctrlb = SERCOM_USART_CTRLB_ENC;
        else
            ctrlb = 0;

        //Calculate baud setting (16x oversampling with fractional baud rate)
        temp = SystemCoreClock * 8 / (16 * arg);

        if ((temp == 0) || (temp > 0xFFFF))
            return ARM_USART_ERROR_BAUDRATE;

        baud = (temp & 0xE0000) | ((temp >> 3) & 0x1FFF);

        break;

    case ARM_USART_MODE_SYNCHRONOUS_MASTER:
        //Check if TXPO setting allows synchronous operation
        if (res->TXPO > 1)
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        //Synchronous mode using internal clock
        ctrla = SERCOM_USART_CTRLA_CMODE | SERCOM_USART_CTRLA_MODE(1);
        ctrlb = 0;

        //Calculate baud setting
        temp = SystemCoreClock / (2 * arg);

        if (temp > 0xFF)
            return ARM_USART_ERROR_BAUDRATE;

        baud = temp & 0xFF;

        break;

    case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
        if (res->TXPO > 1)
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        //Synchronous mode using external clock
        ctrla = SERCOM_USART_CTRLA_CMODE | SERCOM_USART_CTRLA_MODE(0);
        ctrlb = 0;
        break;

    case ARM_USART_CONTROL_RX:
        if (arg == 0)
        {
            usart->CTRLB.bit.RXEN = 0;
            usart->INTENSET.bit.RXC = 0;
        }
        else if (arg == 1)
        {
            usart->CTRLB.bit.RXEN = 1;
            usart->INTENCLR.bit.RXC = 1;
        }
        else
            return ARM_DRIVER_ERROR_PARAMETER;

        //Wait for synchronization of CTRLB register
        while (usart->SYNCBUSY.bit.CTRLB);

        break;

    case ARM_USART_CONTROL_TX:
        if (arg == 0)
            usart->CTRLB.bit.TXEN = 0;
        else if (arg == 1)
            usart->CTRLB.bit.TXEN = 1;
        else
            return ARM_DRIVER_ERROR_PARAMETER;

        //Wait for synchronization of CTRLB register
        while (usart->SYNCBUSY.bit.CTRLB);

        break;

    case ARM_USART_ABORT_RECEIVE:
            if ((res->info->flags & USART_FLAG_TRANSFER) == 0)
            {
                res->info->status.rx_busy = 0;
            }

            break;

    case ARM_USART_ABORT_SEND:
            if ((res->info->flags & USART_FLAG_TRANSFER) == 0)
            {
                usart->INTENCLR.bit.DRE;
                res->info->status.tx_busy = 0;
            }

            break;

    case ARM_USART_ABORT_TRANSFER:
            if (res->info->flags & USART_FLAG_TRANSFER)
            {
                usart->INTENCLR.bit.DRE = 0;

                res->info->status.rx_busy = 0;
                res->info->status.tx_busy = 0;

                res->info->flags &= ~USART_FLAG_TRANSFER;
            }
            break;

    case ARM_USART_SET_DEFAULT_TX_VALUE:
        res->info->defaultTX = (uint8_t) arg;
        break;
// TODO (Ashley Nuttall#1#): Calculate correct RXPL value
//    case ARM_USART_SET_IRDA_PULSE:
//        break;
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    }

    if (ctrla)
    {
        //Check if USART is powered
        if (res->info->flags & ~USART_FLAG_POWERED)
            return ARM_DRIVER_ERROR;

        //Setup pads and set 16x over-sampling using fraction baud generation
        ctrla |= SERCOM_USART_CTRLA_RXPO(res->RXPO) | SERCOM_USART_CTRLA_TXPO(res->TXPO) | SERCOM_USART_CTRLA_SAMPR(1);

        //Set data bits
        switch (control & ARM_USART_DATA_BITS_Msk)
        {
        case ARM_USART_DATA_BITS_5:
            ctrlb |= SERCOM_USART_CTRLB_CHSIZE(5);
            break;

        case ARM_USART_DATA_BITS_6:
            ctrlb |= SERCOM_USART_CTRLB_CHSIZE(6);
            break;

        case ARM_USART_DATA_BITS_7:
            ctrlb |= SERCOM_USART_CTRLB_CHSIZE(7);
            break;

        case ARM_USART_DATA_BITS_8:
            ctrlb |= SERCOM_USART_CTRLB_CHSIZE(0);
            break;

        case ARM_USART_DATA_BITS_9:
            ctrlb |= SERCOM_USART_CTRLB_CHSIZE(1);
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

        if (ctrla && SERCOM_USART_CTRLA_CMODE)
        {
            //Set clock polarity
            if ((control & ARM_USART_CPOL_Msk) == ARM_USART_CPOL1)
                ctrla |= SERCOM_USART_CTRLA_CPOL;

            //Set clock phase
            //if ((control & ARM_USART_CPHA_Msk) == ARM_USART_CPHA1)
            //    ctrla |= SERCOM_USART_CTRLB_CPHA;
        }

        //Set flag for asynchronous or synchronous operation
        if ((control & ARM_USART_CONTROL_Msk) == ARM_USART_MODE_ASYNCHRONOUS)
            res->info->flags |= USART_FLAG_MODE_ASYNCHRONOUS;
        else
            res->info->flags |= USART_FLAG_MODE_SYNCHRONOUS;

        //Disable USART
        if (usart->CTRLA.bit.ENABLE)
        {
            usart->CTRLA.bit.ENABLE = 0;
            while(usart->SYNCBUSY.bit.ENABLE);
        }

        //Set control registers
        usart->CTRLA.reg = ctrla;
        usart->CTRLB.reg = ctrlb;

        //Set baud rate if internal clock is used
        if ((ctrla & SERCOM_USART_CTRLA_MODE_Msk) == 1)
            usart->BAUD.reg = baud;

        //Wait for CTRLB to synchronize
        while (usart->SYNCBUSY.bit.CTRLB);

        //Enable USART
        usart->CTRLA.bit.ENABLE = 1;
        while (usart->SYNCBUSY.bit.ENABLE);
    }

    return ARM_DRIVER_OK;
}


static int32_t USARTx_Send(const USART_RESOURCE *res, const void *data, uint32_t num)
{
    SercomUsart *usart = &res->sercom_res->sercom->USART;

    //Check if USART and transmitter are enabled
    if ((usart->CTRLA.bit.ENABLE == 0)  || (usart->CTRLB.bit.TXEN == 0) || (num == 0))
        return ARM_DRIVER_ERROR;

    if (usart->CTRLA.bit.CMODE)
    {
        //Send using synchronous mode
        return USARTx_Transfer_Synchronous(res, data, 0, num);
    }
    else
    {
        //Check if transmitter is not busy sending
        if (res->info->status.tx_busy == 1)
            return ARM_DRIVER_ERROR_BUSY;

        res->info->status.tx_busy = 1;
        res->info->status.tx_underflow = 0;

        res->info->dataOut = (uint8_t*) data;
        res->info->bytesToSend = num;
        res->info->bytesSent = 0;

        usart->INTENSET.reg = SERCOM_USART_INTENSET_DRE;
    }
    return ARM_DRIVER_OK;
}

static int32_t USARTx_Receive(const USART_RESOURCE *res, void *data, uint32_t num)
{
    SercomUsart *const usart = &res->sercom_res->sercom->USART;

    //Check if USART and receiver is enabled
    if ((usart->CTRLA.bit.ENABLE == 0) || (usart->CTRLB.bit.RXEN == 0) || (num == 0))
        return ARM_DRIVER_ERROR;

    if (usart->CTRLA.bit.CMODE)
    {
        //Receive using synchronous mode
        return USARTx_Transfer_Synchronous(res, 0, data, num);
    }
    else
    {
        //Check if receiver is not busy
        if (res->info->status.rx_busy == 1)
            return ARM_DRIVER_ERROR_BUSY;

        res->info->status.rx_busy = 1;
        res->info->status.rx_break = 0;
        res->info->status.rx_overflow = 0;
        res->info->status.rx_framing_error = 0;
        res->info->status.rx_parity_error = 0;

        res->info->dataIn = (uint8_t*) data;
        res->info->bytesToReceive = num;
        res->info->bytesReceived = 0;
    }

    return ARM_DRIVER_OK;
}

static int32_t USARTx_Transfer(const USART_RESOURCE *res, const void *dataOut, void *dataIn, uint32_t num)
{
    SercomUsart *const usart = &res->sercom_res->sercom->USART;

    //Check if USART, receiver and transmitter is enabled
    if ((usart->CTRLA.bit.ENABLE == 0) || (usart->CTRLB.bit.RXEN == 0) || (usart->CTRLB.bit.TXEN == 0)  || (num == 0))
        return ARM_DRIVER_ERROR;

    return USARTx_Transfer_Synchronous(res, dataOut, dataIn, num);
}


int32_t USARTx_Transfer_Synchronous(const USART_RESOURCE *res, const void *dataOut, void *dataIn, uint32_t num)
{
    SercomUsart *const usart = &res->sercom_res->sercom->USART;

    //Transfer mode is only available in synchronous mode
    if (usart->CTRLA.bit.CMODE != 1)
        return ARM_DRIVER_ERROR;

    //Check if USART is not busy sending or receiving
    if ((res->info->status.tx_busy == 1) || (res->info->status.rx_busy == 1))
        return ARM_DRIVER_ERROR_BUSY;

    res->info->flags |= USART_FLAG_TRANSFER;
    res->info->status.tx_busy = 1;
    res->info->status.rx_busy = 1;
    res->info->dataOut = (uint8_t*) dataOut;
    res->info->dataIn = (uint8_t*) dataIn;
    res->info->bytesToReceive = num;
    res->info->bytesToSend = num;
    res->info->bytesReceived = 0;
    res->info->bytesSent = 0;

    usart->INTENSET.reg = SERCOM_USART_INTENSET_DRE | SERCOM_USART_INTENSET_RXC;

    return ARM_DRIVER_OK;
}

static int32_t USARTx_SetModemControl(const USART_RESOURCE *res, ARM_USART_MODEM_CONTROL control)
{
    return ARM_DRIVER_OK;
}

static ARM_USART_MODEM_STATUS USARTx_GetModemStatus(const USART_RESOURCE *res)
{
    SercomUsart *usart = &res->sercom_res->sercom->USART;
    ARM_USART_MODEM_STATUS retVal;

    retVal.cts = usart->STATUS.bit.CTS;

    return retVal;
}

void USARTx_Handler(const USART_RESOURCE *res)
{
    SercomUsart *usart = &res->sercom_res->sercom->USART;

    if (usart->INTFLAG.bit.DRE)
    {
        if (res->info->status.tx_busy)
        {
            //Send next byte
            if (res->info->dataOut != 0)
                usart->DATA.reg = *res->info->dataOut++;
            else
                usart->DATA.reg = res->info->defaultTX;

            res->info->bytesSent++;

            if (res->info->bytesSent == res->info->bytesToSend)
            {
                //Disable data register empty interrupt
                usart->INTENCLR.bit.DRE = 0;

                //Send complete
                res->info->status.tx_busy = 0;

                //Signal send complete event
                if ((res->info->cb_event) && ((res->info->flags & USART_FLAG_TRANSFER) == 0))
                    res->info->cb_event(ARM_USART_EVENT_SEND_COMPLETE);
            }
        }
        else
        {
            usart->INTENCLR.bit.DRE = 0;
        }
    }
    else if (usart->INTFLAG.bit.RXC)
    {
        if (res->info->status.rx_busy)
        {
            uint8_t data = usart->DATA.reg;

            if (res->info->dataIn)
                *res->info->dataIn++ = data;

            res->info->bytesReceived++;

            if (res->info->bytesReceived == res->info->bytesToReceive)
            {
                //Receive complete
                res->info->status.rx_busy = 0;

                //Signal receive complete event
                if (res->info->cb_event)
                {
                    if (res->info->flags & USART_FLAG_TRANSFER)
                    {
                        res->info->flags &= ~USART_FLAG_TRANSFER;
                        res->info->cb_event(ARM_USART_EVENT_TRANSFER_COMPLETE);
                    }
                    else
                        res->info->cb_event(ARM_USART_EVENT_RECEIVE_COMPLETE);
                }
            }
        }
        else
        {
            //Receive buffer overflow
            res->info->status.rx_overflow = 1;

            //Signal buffer overflow event
            if (res->info->cb_event)
                res->info->cb_event(ARM_USART_EVENT_RX_OVERFLOW);
        }
    }
    else if (usart->INTFLAG.bit.RXBRK)
    {
        //Receive break detected
        res->info->status.rx_break = 1;

        //Signal event
        if (res->info->cb_event)
            res->info->cb_event(ARM_USART_EVENT_RX_BREAK);

        //Clear receive break flag
        usart->INTFLAG.reg |= SERCOM_USART_INTFLAG_RXBRK;
    }
    else if (usart->INTFLAG.bit.ERROR)
    {
        if (usart->STATUS.bit.FERR)
        {
            //Framing error occurred
            res->info->status.rx_framing_error = 1;

            //Signal event
            if (res->info->cb_event)
                res->info->cb_event(ARM_USART_EVENT_RX_FRAMING_ERROR);

            //Clear framing error flag
            usart->STATUS.reg = SERCOM_USART_STATUS_FERR;
        }
        else if (usart->STATUS.bit.PERR)
        {
            //Parity error occurred
            res->info->status.rx_parity_error = 1;

            //Signal event
            if (res->info->cb_event)
                res->info->cb_event(ARM_USART_EVENT_RX_PARITY_ERROR);

            //Clear parity error flag
            usart->STATUS.reg = SERCOM_USART_STATUS_PERR;
        }
        else if (usart->STATUS.bit.BUFOVF)
        {
            //Receiver overflow occurred
            res->info->status.rx_overflow = 1;

            //Signal event
            if (res->info->cb_event)
                res->info->cb_event(ARM_USART_EVENT_RX_OVERFLOW);

            //Clear overflow flag
            usart->STATUS.reg = SERCOM_USART_STATUS_BUFOVF;
        }

        //Reset error flag
        usart->INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;
    }
}

//Configure USART1
#if (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_USART)
static USART_INFO USART1_info = {0, {0}, 0, 0, 0, 0, 0, 0xFF};
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
static uint32_t USART1_GetTxCount()                                                 { return USART1_res.info->bytesSent;}
static uint32_t USART1_GetRxCount()                                                 { return USART1_res.info->bytesReceived;}
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

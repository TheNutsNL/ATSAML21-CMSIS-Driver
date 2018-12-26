#include "SPI_SAML21.h"
#include "SERCOM_SAML21.h"
#include "RTE_Device.h"

#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) //Driver version

extern uint32_t SystemCoreClock;

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_SPI_API_VERSION,
    ARM_SPI_DRV_VERSION
};

//Driver Capabilities
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
    0, //Simplex Mode (Master and Slave)
    0, //TI Synchronous Serial Interface
    0, //Microwire Interface
    0  //Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT
};

ARM_DRIVER_VERSION ARM_SPI_GetVersion(void)
{
    return DriverVersion;
}

ARM_SPI_CAPABILITIES ARM_SPI_GetCapabilities(void)
{
    return DriverCapabilities;
}

static uint32_t SPIx_Initialize(const SPI_RESOURCE *res, ARM_SPI_SignalEvent_t cb_event)
{
    if (res->info->flags & SPI_FLAG_INITIALIZED)
        return ARM_DRIVER_OK;

    //Set callback event
    res->info->cb_event = cb_event;
    res->info->status.busy = 0;
    res->info->status.data_lost = 0;
    res->info->status.mode_fault = 0;

    //Enable output pins
    switch (res->DOPO)
    {
    case 0: //DO: pad0, SCK: pad1, SS: pad2
        PinEnable(res->sercom_res->pad[0]);
        PinEnable(res->sercom_res->pad[1]);
        break;

    case 1: //DO: pad2, SCK: pad3, SS: pad1
        PinEnable(res->sercom_res->pad[2]);
        PinEnable(res->sercom_res->pad[3]);
        break;

    case 2: //DO: pad3, SCK: pad1, SS: pad2
        PinEnable(res->sercom_res->pad[3]);
        PinEnable(res->sercom_res->pad[1]);
        break;

    case 3: //DO: pad0, SCK: pad3, SS: pad1
        PinEnable(res->sercom_res->pad[0]);
        PinEnable(res->sercom_res->pad[3]);
        break;
    }

    //Enable input pin
    PinEnable(res->sercom_res->pad[res->DIPO]);

    //Set initialized flag
    res->info->flags = SPI_FLAG_INITIALIZED;

    return ARM_DRIVER_OK;
}

static uint32_t SPIx_Unintialize(const SPI_RESOURCE *res)
{
    //Power of SPI if powered on
    if (res->info->flags & SPI_FLAG_POWERED)
        SPIx_PowerControl(res, ARM_POWER_OFF);

    //Disable output pins
    switch (res->DOPO)
    {
    case 0: //DO: pad0, SCK: pad1, SS: pad2
        PinDisable(res->sercom_res->pad[0]);
        PinDisable(res->sercom_res->pad[1]);
        break;

    case 1: //DO: pad2, SCK: pad3, SS: pad1
        PinDisable(res->sercom_res->pad[2]);
        PinDisable(res->sercom_res->pad[3]);
        break;

    case 2: //DO: pad3, SCK: pad1, SS: pad2
        PinDisable(res->sercom_res->pad[3]);
        PinDisable(res->sercom_res->pad[1]);
        break;

    case 3: //DO: pad0, SCK: pad3, SS: pad1
        PinDisable(res->sercom_res->pad[0]);
        PinDisable(res->sercom_res->pad[3]);
        break;
    }

    //Enable input pin
    PinDisable(res->sercom_res->pad[res->DIPO]);

    //Clear flags
    res->info->flags = 0;

    return ARM_DRIVER_OK;
}

static int32_t SPIx_PowerControl(const SPI_RESOURCE *res, ARM_POWER_STATE state)
{
    switch (state)
    {
    case ARM_POWER_OFF:
        //Disable SPI if enabled
        if (res->info->flags & SPI_FLAG_ENABLED)
            SPIx_Control(res, ARM_SPI_MODE_INACTIVE, 0);

        //Disable interrupt
        NVIC_DisableIRQ(res->sercom_res->irq);

        //Disable peripheral clock
        Sercom_PeriphalClockDisable(res->sercom_res);

        res->info->status.busy = 0;
        res->info->status.data_lost = 0;
        res->info->status.mode_fault = 0;

        //Clear powered flag
        res->info->flags &= ~SPI_FLAG_POWERED;

        break;

//    case ARM_POWER_LOW:
//        break;

    case ARM_POWER_FULL:
        if ((res->info->flags & SPI_FLAG_INITIALIZED )== 0)
            return ARM_DRIVER_ERROR;

        if (res->info->flags & SPI_FLAG_POWERED)
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

static int32_t SPIx_Control(const SPI_RESOURCE *res, uint32_t control, uint32_t arg)
{
    SercomSpi *const spi = &res->sercom_res->sercom->SPI;
    const uint32_t pinSS = res->sercom_res->pad[2 - (res->DOPO & 0x1)];
    uint32_t temp;

    switch (control & ARM_SPI_CONTROL_Msk)
    {
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    //SPI Inactive
    case ARM_SPI_MODE_INACTIVE:
        if (res->info->flags & SPI_FLAG_ENABLED)
        {
            //Disable software or hardware controlled SS pin
            if (res->info->flags & SPI_FLAG_SS_SOFTWARE)
                PORT->Group[DRIVER_GET_PER_PORT(pinSS)].DIRSET.reg = (1 << DRIVER_GET_PER_PIN(pinSS));
            else if (res->info->flags & SPI_FLAG_SS_HARDWARE)
                PinDisable(pinSS);

            //Disable SPI
            spi->CTRLA.bit.ENABLE = 0;
            while (spi->SYNCBUSY.bit.ENABLE);

            res->info->flags = SPI_FLAG_INITIALIZED | SPI_FLAG_POWERED;
        }
        break;

    //Enable master mode
    case ARM_SPI_MODE_MASTER:
        if ((res->info->flags & SPI_FLAG_POWERED) == 0)
            return ARM_DRIVER_ERROR;

        if ((control & ARM_SPI_DATA_BITS_Msk) != (8 << ARM_SPI_DATA_BITS_Pos))
            return ARM_SPI_ERROR_DATA_BITS;

        //Reset SERCOM
        spi->CTRLA.bit.SWRST = 1;
        while (spi->SYNCBUSY.bit.SWRST);

        //Setup SPI control register A
        temp = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DOPO(res->DOPO) | SERCOM_SPI_CTRLA_DIPO(res->DIPO);

        //Set frame format
        switch (control & ARM_SPI_FRAME_FORMAT_Msk)
        {
        case ARM_SPI_CPOL0_CPHA0:
            break;

        case ARM_SPI_CPOL0_CPHA1:
            temp |= SERCOM_SPI_CTRLA_CPHA;
            break;

        case ARM_SPI_CPOL1_CPHA0:
            temp |= SERCOM_SPI_CTRLA_CPOL;
            break;

        case ARM_SPI_CPOL1_CPHA1:
            temp |= SERCOM_SPI_CTRLA_CPHA | SERCOM_SPI_CTRLA_CPOL;
            break;

        default:
            return ARM_SPI_ERROR_FRAME_FORMAT;
        }

        //Set data order
        if (control & ARM_SPI_LSB_MSB)
            temp |= SERCOM_SPI_CTRLA_DORD;

        spi->CTRLA.reg = temp;

        //Setup SPI control register B
        temp = SERCOM_SPI_CTRLB_RXEN;

        //SS pin control
        switch (control & ARM_SPI_SS_MASTER_MODE_Msk)
        {
        case ARM_SPI_SS_MASTER_SW:
            //Set up pin for output
            PinDisable(pinSS);
            PORT->Group[DRIVER_GET_PER_PORT(pinSS)].OUTSET.reg = (1 << DRIVER_GET_PER_PIN(pinSS));
            PORT->Group[DRIVER_GET_PER_PORT(pinSS)].DIRSET.reg = (1 << DRIVER_GET_PER_PIN(pinSS));

            res->info->flags |= SPI_FLAG_SS_SOFTWARE;
            break;

        case ARM_SPI_SS_MASTER_HW_INPUT:
            temp |= SERCOM_SPI_CTRLB_SSDE;

        case ARM_SPI_SS_MASTER_HW_OUTPUT:
            temp |= SERCOM_SPI_CTRLB_MSSEN;
            PinEnable(pinSS);

            res->info->flags |= SPI_FLAG_SS_HARDWARE;
            break;
        }
        spi->CTRLB.reg = temp;

        //Set baud rate
        spi->BAUD.reg = SystemCoreClock / (2 * arg) - 1;
        while (spi->SYNCBUSY.bit.CTRLB);

        //Enable SPI
        spi->CTRLA.bit.ENABLE = 1;
        while (spi->SYNCBUSY.bit.ENABLE);

        res->info->flags |= SPI_FLAG_ENABLED;

        break;

    //case ARM_SPI_MODE_SLAVE:                // SPI Slave  (Output on MISO, Input on MOSI)
    //case ARM_SPI_MODE_MASTER_SIMPLEX:       // SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
    //case ARM_SPI_MODE_SLAVE_SIMPLEX:        // SPI Slave  (Output/Input on MISO)

    //Set baud rate
    case ARM_SPI_SET_BUS_SPEED:
        if (res->info->flags & SPI_FLAG_ENABLED)
        {
            //Disable SPI
            spi->CTRLA.bit.ENABLE = 0;
            while (spi->SYNCBUSY.bit.ENABLE);

            //Set baud rate
            spi->BAUD.reg = SystemCoreClock / (2 * arg) - 1;

            //Enable SPI
            spi->CTRLA.bit.ENABLE = 1;
            while (spi->SYNCBUSY.bit.ENABLE);
        }
        break;

    //Get baud rate
    case ARM_SPI_GET_BUS_SPEED:
        return SystemCoreClock / (2 * (spi->BAUD.reg + 1));

    case ARM_SPI_SET_DEFAULT_TX_VALUE:      // Set default Transmit value; arg = value
        res->info->defaultTX = (uint8_t) arg;
        break;

    //Control Slave Select
    case ARM_SPI_CONTROL_SS:
        if ((res->info->flags && SPI_FLAG_SS_SOFTWARE) == 0)
            return ARM_DRIVER_ERROR;

        if (arg)
            PORT->Group[DRIVER_GET_PER_PORT(pinSS)].OUTCLR.reg = (1 << DRIVER_GET_PER_PIN(pinSS));
        else
            PORT->Group[DRIVER_GET_PER_PORT(pinSS)].OUTSET.reg = (1 << DRIVER_GET_PER_PIN(pinSS));

        break;

    //Abort current data transfer
    case ARM_SPI_ABORT_TRANSFER:
        break;
    }

    return ARM_DRIVER_OK;
}

static uint32_t SPIx_TransferData(const SPI_RESOURCE *res, const void *dataOut, void *dataIn, uint16_t dataLength)
{
    SercomSpi *const spi = &res->sercom_res->sercom->SPI;

    if ((res->info->flags & SPI_FLAG_ENABLED) == 0)
        return ARM_DRIVER_ERROR;

    if (res->info->status.busy)
        return ARM_DRIVER_ERROR_BUSY;

    res->info->status.busy = 1;
    res->info->status.data_lost = 0;
    res->info->status.mode_fault= 0;

    res->info->dataOut = (uint8_t*) dataOut;
    res->info->dataIn = (uint8_t*) dataIn;
    res->info->bytesToTransfer = dataLength;
    res->info->bytesRecieved = 0;
    res->info->bytesSent = 0;

    spi->INTENSET.reg = SERCOM_SPI_INTENSET_DRE;
    spi->INTENSET.reg = SERCOM_SPI_INTENSET_RXC;

    return ARM_DRIVER_OK;
}

void SPIx_Handler(const SPI_RESOURCE *res)
{
    SercomSpi *const spi = &res->sercom_res->sercom->SPI;
    SPI_INFO *const ti = res->info;

    //Data register empty interrupt
    if (spi->INTFLAG.bit.DRE)
    {
        //Send next byte
        if (ti->dataOut)
            spi->DATA.reg = *ti->dataOut++;
        else
            spi->DATA.reg = 0xFF;

        ti->bytesSent++;
        if (ti->bytesSent == ti->bytesToTransfer)
            //Disable data register empty interrupt flag
            spi->INTENCLR.reg = SERCOM_SPI_INTENCLR_DRE;
    }

    //Receive complete interrupt
    if (spi->INTFLAG.bit.RXC)
    {
        if (ti->dataIn)
            *ti->dataIn++ = spi->DATA.reg;
        else
            (uint8_t) spi->DATA.reg;

        ti->bytesRecieved++;
        if (ti->bytesRecieved == ti->bytesToTransfer)
        {
            //Disable receive complete interrupt
            spi->INTENCLR.reg = SERCOM_SPI_INTENCLR_RXC;

            ti->status.busy = 0;

            //Call callback event
            ti->cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
        }
    }
}

//Configure SPI0
#if (RTE_SERCOM0_MODE == RTE_SERCOM_MODE_SPI)
static SPI_INFO SPI0_info = {0, {0}, 0, 0, 0, 0, 0, 0, 0xFF};
static const SPI_RESOURCE SPI0_res =
{
    &sercom0_res,
    RTE_SPI0_DOPO,
    RTE_SPI0_DIPO,
    &SPI0_info
};

void SERCOM0_Handler() {SPIx_Handler(&SPI0_res);}
static int32_t ARM_SPI0_Initialize(ARM_SPI_SignalEvent_t cb_event) {MCLK->APBCMASK.bit.SERCOM0_ = 1; return SPIx_Initialize(&SPI0_res, cb_event); }
static int32_t ARM_SPI0_Uninitialize(void) { return SPIx_Unintialize(&SPI0_res);};
static int32_t ARM_SPI0_PowerControl(ARM_POWER_STATE state) { return SPIx_PowerControl(&SPI0_res, state); }
static int32_t ARM_SPI0_Send(const void *data, uint32_t num) { return SPIx_TransferData(&SPI0_res, data, 0, num); }
static int32_t ARM_SPI0_Receive(void *data, uint32_t num) { return SPIx_TransferData(&SPI0_res, 0, data, num); }
static int32_t ARM_SPI0_Transfer(const void *data_out, void *data_in, uint32_t num) { return SPIx_TransferData(&SPI0_res, data_out, data_in, num);}
static uint32_t ARM_SPI0_GetDataCount(void) { return SPI0_res.info->bytesRecieved; }
static int32_t ARM_SPI0_Control(uint32_t control, uint32_t arg) { return SPIx_Control(&SPI0_res, control, arg);}
static ARM_SPI_STATUS ARM_SPI0_GetStatus() { return SPI0_res.info->status;}

ARM_DRIVER_SPI Driver_SPI0 =
{
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI0_Initialize,
    ARM_SPI0_Uninitialize,
    ARM_SPI0_PowerControl,
    ARM_SPI0_Send,
    ARM_SPI0_Receive,
    ARM_SPI0_Transfer,
    ARM_SPI0_GetDataCount,
    ARM_SPI0_Control,
    ARM_SPI0_GetStatus
};
#endif

//Configure SPI1
#if (RTE_SERCOM1_MODE == RTE_SERCOM_MODE_SPI)
static SPI_INFO SPI1_info = {0, {0}, 0, 0, 0, 0, 0, 0, 0xFF};
static const SPI_RESOURCE SPI1_res =
{
    &sercom1_res,
    RTE_SPI1_DOPO,
    RTE_SPI1_DIPO,
    &SPI1_info
};

void SERCOM1_Handler() {SPIx_Handler(&SPI1_res);}
static int32_t ARM_SPI1_Initialize(ARM_SPI_SignalEvent_t cb_event) {MCLK->APBCMASK.bit.SERCOM1_ = 1; return SPIx_Initialize(&SPI1_res, cb_event); }
static int32_t ARM_SPI1_Uninitialize(void) { return SPIx_Unintialize(&SPI1_res);};
static int32_t ARM_SPI1_PowerControl(ARM_POWER_STATE state) { return SPIx_PowerControl(&SPI1_res, state); }
static int32_t ARM_SPI1_Send(const void *data, uint32_t num) { return SPIx_TransferData(&SPI1_res, data, 0, num); }
static int32_t ARM_SPI1_Receive(void *data, uint32_t num) { return SPIx_TransferData(&SPI1_res, 0, data, num); }
static int32_t ARM_SPI1_Transfer(const void *data_out, void *data_in, uint32_t num) { return SPIx_TransferData(&SPI1_res, data_out, data_in, num);}
static uint32_t ARM_SPI1_GetDataCount(void) { return SPI1_res.info->bytesRecieved; }
static int32_t ARM_SPI1_Control(uint32_t control, uint32_t arg) { return SPIx_Control(&SPI1_res, control, arg);}
static ARM_SPI_STATUS ARM_SPI1_GetStatus() { return SPI1_res.info->status;}

ARM_DRIVER_SPI Driver_SPI1 =
{
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI1_Initialize,
    ARM_SPI1_Uninitialize,
    ARM_SPI1_PowerControl,
    ARM_SPI1_Send,
    ARM_SPI1_Receive,
    ARM_SPI1_Transfer,
    ARM_SPI1_GetDataCount,
    ARM_SPI1_Control,
    ARM_SPI1_GetStatus
};
#endif


//Configure SPI2
#if (RTE_SERCOM2_MODE == RTE_SERCOM_MODE_SPI)
static SPI_INFO SPI2_info = {0, {0}, 0, 0, 0, 0, 0, 0, 0xFF};
static const SPI_RESOURCE SPI2_res =
{
    &sercom2_res,
    RTE_SPI2_DOPO,
    RTE_SPI2_DIPO,
    &SPI2_info
};

void SERCOM2_Handler() {SPIx_Handler(&SPI2_res);}
static int32_t ARM_SPI2_Initialize(ARM_SPI_SignalEvent_t cb_event) {MCLK->APBCMASK.bit.SERCOM2_ = 1; return SPIx_Initialize(&SPI2_res, cb_event); }
static int32_t ARM_SPI2_Uninitialize(void) { return SPIx_Unintialize(&SPI2_res);};
static int32_t ARM_SPI2_PowerControl(ARM_POWER_STATE state) { return SPIx_PowerControl(&SPI2_res, state); }
static int32_t ARM_SPI2_Send(const void *data, uint32_t num) { return SPIx_TransferData(&SPI2_res, data, 0, num); }
static int32_t ARM_SPI2_Receive(void *data, uint32_t num) { return SPIx_TransferData(&SPI2_res, 0, data, num); }
static int32_t ARM_SPI2_Transfer(const void *data_out, void *data_in, uint32_t num) { return SPIx_TransferData(&SPI2_res, data_out, data_in, num);}
static uint32_t ARM_SPI2_GetDataCount(void) { return SPI2_res.info->bytesRecieved; }
static int32_t ARM_SPI2_Control(uint32_t control, uint32_t arg) { return SPIx_Control(&SPI2_res, control, arg);}
static ARM_SPI_STATUS ARM_SPI2_GetStatus() { return SPI2_res.info->status;}

ARM_DRIVER_SPI Driver_SPI2 =
{
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI2_Initialize,
    ARM_SPI2_Uninitialize,
    ARM_SPI2_PowerControl,
    ARM_SPI2_Send,
    ARM_SPI2_Receive,
    ARM_SPI2_Transfer,
    ARM_SPI2_GetDataCount,
    ARM_SPI2_Control,
    ARM_SPI2_GetStatus
};
#endif


//Configure SPI3
#if (RTE_SERCOM3_MODE == RTE_SERCOM_MODE_SPI)
static SPI_INFO SPI3_info = {0, {0}, 0, 0, 0, 0, 0, 0, 0xFF};
static const SPI_RESOURCE SPI3_res =
{
    &sercom3_res,
    RTE_SPI3_DOPO,
    RTE_SPI3_DIPO,
    &SPI3_info
};

void SERCOM3_Handler() {SPIx_Handler(&SPI3_res);}
static int32_t ARM_SPI3_Initialize(ARM_SPI_SignalEvent_t cb_event) {MCLK->APBCMASK.bit.SERCOM3_ = 1; return SPIx_Initialize(&SPI3_res, cb_event); }
static int32_t ARM_SPI3_Uninitialize(void) { return SPIx_Unintialize(&SPI3_res);};
static int32_t ARM_SPI3_PowerControl(ARM_POWER_STATE state) { return SPIx_PowerControl(&SPI3_res, state); }
static int32_t ARM_SPI3_Send(const void *data, uint32_t num) { return SPIx_TransferData(&SPI3_res, data, 0, num); }
static int32_t ARM_SPI3_Receive(void *data, uint32_t num) { return SPIx_TransferData(&SPI3_res, 0, data, num); }
static int32_t ARM_SPI3_Transfer(const void *data_out, void *data_in, uint32_t num) { return SPIx_TransferData(&SPI3_res, data_out, data_in, num);}
static uint32_t ARM_SPI3_GetDataCount(void) { return SPI3_res.info->bytesRecieved; }
static int32_t ARM_SPI3_Control(uint32_t control, uint32_t arg) { return SPIx_Control(&SPI3_res, control, arg);}
static ARM_SPI_STATUS ARM_SPI3_GetStatus() { return SPI3_res.info->status;}

ARM_DRIVER_SPI Driver_SPI3 =
{
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI3_Initialize,
    ARM_SPI3_Uninitialize,
    ARM_SPI3_PowerControl,
    ARM_SPI3_Send,
    ARM_SPI3_Receive,
    ARM_SPI3_Transfer,
    ARM_SPI3_GetDataCount,
    ARM_SPI3_Control,
    ARM_SPI3_GetStatus
};
#endif


//Configure SPI4
#if (RTE_SERCOM4_MODE == RTE_SERCOM_MODE_SPI)
static SPI_INFO SPI4_info = {0, {0}, 0, 0, 0, 0, 0, 0, 0xFF};
static const SPI_RESOURCE SPI4_res =
{
    &sercom4_res,
    RTE_SPI4_DOPO,
    RTE_SPI4_DIPO,
    &SPI4_info
};

void SERCOM4_Handler() {SPIx_Handler(&SPI4_res);}
static int32_t ARM_SPI4_Initialize(ARM_SPI_SignalEvent_t cb_event) {MCLK->APBCMASK.bit.SERCOM4_ = 1; return SPIx_Initialize(&SPI4_res, cb_event); }
static int32_t ARM_SPI4_Uninitialize(void) { return SPIx_Unintialize(&SPI4_res);};
static int32_t ARM_SPI4_PowerControl(ARM_POWER_STATE state) { return SPIx_PowerControl(&SPI4_res, state); }
static int32_t ARM_SPI4_Send(const void *data, uint32_t num) { return SPIx_TransferData(&SPI4_res, data, 0, num); }
static int32_t ARM_SPI4_Receive(void *data, uint32_t num) { return SPIx_TransferData(&SPI4_res, 0, data, num); }
static int32_t ARM_SPI4_Transfer(const void *data_out, void *data_in, uint32_t num) { return SPIx_TransferData(&SPI4_res, data_out, data_in, num);}
static uint32_t ARM_SPI4_GetDataCount(void) { return SPI4_res.info->bytesRecieved; }
static int32_t ARM_SPI4_Control(uint32_t control, uint32_t arg) { return SPIx_Control(&SPI4_res, control, arg);}
static ARM_SPI_STATUS ARM_SPI4_GetStatus() { return SPI4_res.info->status;}

ARM_DRIVER_SPI Driver_SPI4 =
{
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI4_Initialize,
    ARM_SPI4_Uninitialize,
    ARM_SPI4_PowerControl,
    ARM_SPI4_Send,
    ARM_SPI4_Receive,
    ARM_SPI4_Transfer,
    ARM_SPI4_GetDataCount,
    ARM_SPI4_Control,
    ARM_SPI4_GetStatus
};
#endif


//Configure SPI5
#if (RTE_SERCOM5_MODE == RTE_SERCOM_MODE_SPI)
static SPI_INFO SPI5_info = {0, {0}, 0, 0, 0, 0, 0, 0, 0xFF};
static const SPI_RESOURCE SPI5_res =
{
    &sercom5_res,
    RTE_SPI5_DOPO,
    RTE_SPI5_DIPO,
    &SPI5_info,
};

void SERCOM5_Handler() {SPIx_Handler(&SPI5_res);}
static int32_t ARM_SPI5_Initialize(ARM_SPI_SignalEvent_t cb_event) {MCLK->APBCMASK.bit.SERCOM5_ = 1; return SPIx_Initialize(&SPI5_res, cb_event); }
static int32_t ARM_SPI5_Uninitialize(void) { return SPIx_Unintialize(&SPI5_res);};
static int32_t ARM_SPI5_PowerControl(ARM_POWER_STATE state) { return SPIx_PowerControl(&SPI5_res, state); }
static int32_t ARM_SPI5_Send(const void *data, uint32_t num) { return SPIx_TransferData(&SPI5_res, data, 0, num); }
static int32_t ARM_SPI5_Receive(void *data, uint32_t num) { return SPIx_TransferData(&SPI5 res, 0, data, num); }
static int32_t ARM_SPI5_Transfer(const void *data_out, void *data_in, uint32_t num) { return SPIx_TransferData(&SPI5_res, data_out, data_in, num);}
static uint32_t ARM_SPI5_GetDataCount(void) { return SPI5_res.info->bytesRecieved; }
static int32_t ARM_SPI5_Control(uint32_t control, uint32_t arg) { return SPIx_Control(&SPI5_res, control, arg);}
static ARM_SPI_STATUS ARM_SPI5_GetStatus() { return SPI5_res.info->status;}

ARM_DRIVER_SPI Driver_SPI5 =
{
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI5_Initialize,
    ARM_SPI5_Uninitialize,
    ARM_SPI5_PowerControl,
    ARM_SPI5_Send,
    ARM_SPI5_Receive,
    ARM_SPI5_Transfer,
    ARM_SPI5_GetDataCount,
    ARM_SPI5_Control,
    ARM_SPI5_GetStatus
};
#endif

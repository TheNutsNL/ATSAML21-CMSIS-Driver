/*
 * $projectname$.c
 *
 * Created: 8-4-2018
 * Author : Ashley Nuttall
 */

#include "sam.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Driver_SPI.h"
#include "Driver_USART.h"

#define LED_PIN PORT_PA27

#define PRIORITY_TASK_BLINK tskIDLE_PRIORITY + 1

extern ARM_DRIVER_SPI Driver_SPI0;
extern ARM_DRIVER_USART Driver_USART1;

bool isSpiBusy = false;
bool isUsartBusy = false;

void vApplicationIdleHook(void);
void TaskBlink(void *param);
void TestSPI();
void TestUSART();

int main(void)
{

    /* Initialize the SAM system */
    SystemInit();

    xTaskCreate(TaskBlink, "Blink", configMINIMAL_STACK_SIZE, NULL, PRIORITY_TASK_BLINK, NULL);
    vTaskStartScheduler();

    while (1)
    {
    }
}


void TaskBlink(void *param)
{
    PORT->Group[0].DIRSET.reg = LED_PIN;
    PORT->Group[0].OUTSET.reg = LED_PIN;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        PORT->Group[0].OUTTGL.reg = LED_PIN;

        TestSPI();
        TestUSART();
    }
}

void SPI_Callback(uint32_t event)
{
    switch (event)
    {
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
        isSpiBusy = false;
        break;
    case ARM_SPI_EVENT_DATA_LOST:
        break;
    case ARM_SPI_EVENT_MODE_FAULT:
        break;
    }
}

void TestSPI()
{
    const ARM_DRIVER_SPI *spi = &Driver_SPI0;
    uint8_t dataOut[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint8_t dataIn[8];

    spi->Initialize(SPI_Callback);
    spi->PowerControl(ARM_POWER_FULL);
    spi->Control(ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8), 100000);

    //Send data
    spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    isSpiBusy = true;
    spi->Send(dataOut, sizeof(dataOut));
    while (isSpiBusy);
    spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

    //Clear data in buffer
    dataIn[0] = 0xFF; dataIn[1] = 0xFF; dataIn[2] = 0xFF; dataIn[3] = 0xFF;
    dataIn[4] = 0xFF; dataIn[5] = 0xFF; dataIn[6] = 0xFF; dataIn[7] = 0xFF;

    //Receive data
    spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    isSpiBusy = true;
    spi->Receive(dataIn, sizeof(dataIn));
    while (isSpiBusy);
    spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

    //Clear data in buffer
    dataIn[0] = 0xFF; dataIn[1] = 0xFF; dataIn[2] = 0xFF; dataIn[3] = 0xFF;
    dataIn[4] = 0xFF; dataIn[5] = 0xFF; dataIn[6] = 0xFF; dataIn[7] = 0xFF;

    //Send and receive data
    spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    isSpiBusy = true;
    spi->Transfer(dataOut, dataIn, sizeof(dataIn));
    while (isSpiBusy);
    spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

}

void USART_Callback(uint32_t event)
{
    isUsartBusy = false;
}

void TestUSART()
{
    const ARM_DRIVER_USART *usart = &Driver_USART1;
    uint8_t dataOut[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    //uint8_t dataIn[8];

    usart->Initialize(USART_Callback);
    usart->PowerControl(ARM_POWER_FULL);

    usart->Control(ARM_USART_MODE_ASYNCHRONOUS, 9600);
    isUsartBusy = false;
    usart->Send(dataOut, sizeof(dataOut));
    while (isUsartBusy);

    usart->PowerControl(ARM_POWER_OFF);
    usart->Uninitialize();
}

void vApplicationIdleHook(void)
{
    __WFI();
}

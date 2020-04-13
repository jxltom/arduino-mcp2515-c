#include "config.h"
#if defined(SPI_CC2640)

#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include "../spi.h"

static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;
static PIN_Handle spiCsPin = NULL;
static PIN_State spiCsPinState;
static PIN_Config spiCsPinConfigTable[] = {
    SPICS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN, PIN_TERMINATE};


void spi_init(void)
{
    SPI_init();

    // Configure SPI as master
    SPI_Params_init(&spiParams);
    spiParams.bitRate = SPI_CLOCK;
    spiParams.mode = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_BLOCKING;

    // Attempt to open SPI
    spiHandle = SPI_open(0, &spiParams);
    if (spiHandle == NULL)
    {
        while (1);
    }

    spiCsPin = PIN_open(&spiCsPinState, spiCsPinConfigTable);
}

unsigned char spi_transfer(const unsigned char data)
{
    SPI_Transaction masterTransaction;
    uint8_t receiveData = 0;
    masterTransaction.count = SPI_TRANSFER_LEN;
    masterTransaction.txBuf = (void *)&data;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = &receiveData;
    if (SPI_transfer(spiHandle, &masterTransaction))
    {
        return receiveData;
    }
    return 0;
}

void spi_end()
{
    PIN_setOutputValue(spiCsPin, SPICS, 1);
}

void spi_start()
{
    PIN_setOutputValue(spiCsPin, SPICS, 0);
}

#endif

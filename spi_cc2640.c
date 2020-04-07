#include "spi_cc2640.h"

static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;
static PIN_Handle spiCsPin = NULL;
static PIN_State spiCsPinState;
static PIN_Config spiCsPinConfigTable[] = {
    SPICS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN, PIN_TERMINATE};


void spi_begin(void)
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

uint8_t spi_transfer(const uint8_t *transmitBufferPointer)
{
    SPI_Transaction masterTransaction;
    uint8_t receiveBufferPointer = 0;
    masterTransaction.count = SPI_TRANSFER_LEN;
    masterTransaction.txBuf = transmitBufferPointer;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = receiveBufferPointer;
    if (SPI_transfer(spiHandle, &masterTransaction))
    {
        return receiveBufferPointer
    }
    return 0
}

void spi_end()
{
    PIN_setOutputValue(spiCsPin, SPICS, 1);
}

void spi_start()
{
    PIN_setOutputValue(spiCsPin, SPICS, 0);
}

#include "Arduino.h"
#include "mcp2515.h"

static const uint8_t CANCTRL_REQOP = 0xE0;
static const uint8_t CANCTRL_ABAT = 0x10;
static const uint8_t CANCTRL_OSM = 0x08;
static const uint8_t CANCTRL_CLKEN = 0x04;
static const uint8_t CANCTRL_CLKPRE = 0x03;

typedef enum
{
    CANCTRL_REQOP_NORMAL = 0x00,
    CANCTRL_REQOP_SLEEP = 0x20,
    CANCTRL_REQOP_LOOPBACK = 0x40,
    CANCTRL_REQOP_LISTENONLY = 0x60,
    CANCTRL_REQOP_CONFIG = 0x80,
    CANCTRL_REQOP_POWERUP = 0xE0
} CANCTRL_REQOP_MODE;

static const uint8_t CANSTAT_OPMOD = 0xE0;
static const uint8_t CANSTAT_ICOD = 0x0E;

static const uint8_t CNF3_SOF = 0x80;

static const uint8_t TXB_EXIDE_MASK = 0x08;
static const uint8_t DLC_MASK = 0x0F;
static const uint8_t RTR_MASK = 0x40;

static const uint8_t RXBnCTRL_RXM_STD = 0x20;
static const uint8_t RXBnCTRL_RXM_EXT = 0x40;
static const uint8_t RXBnCTRL_RXM_STDEXT = 0x00;
static const uint8_t RXBnCTRL_RXM_MASK = 0x60;
static const uint8_t RXBnCTRL_RTR = 0x08;
static const uint8_t RXB0CTRL_BUKT = 0x04;
static const uint8_t RXB0CTRL_FILHIT_MASK = 0x03;
static const uint8_t RXB1CTRL_FILHIT_MASK = 0x07;
static const uint8_t RXB0CTRL_FILHIT = 0x00;
static const uint8_t RXB1CTRL_FILHIT = 0x01;

static const uint8_t MCP_SIDH = 0;
static const uint8_t MCP_SIDL = 1;
static const uint8_t MCP_EID8 = 2;
static const uint8_t MCP_EID0 = 3;
static const uint8_t MCP_DLC = 4;
static const uint8_t MCP_DATA = 5;

typedef enum
{
    STAT_RX0IF = (1 << 0),
    STAT_RX1IF = (1 << 1)
} STAT;

static const uint8_t STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF;

typedef enum
{
    TXB_ABTF = 0x40,
    TXB_MLOA = 0x20,
    TXB_TXERR = 0x10,
    TXB_TXREQ = 0x08,
    TXB_TXIE = 0x04,
    TXB_TXP = 0x03
} TXBnCTRL;

static const uint8_t EFLG_ERRORMASK = EFLG_RX1OVR | EFLG_RX0OVR | EFLG_TXBO | EFLG_TXEP | EFLG_RXEP;

typedef enum
{
    INSTRUCTION_WRITE = 0x02,
    INSTRUCTION_READ = 0x03,
    INSTRUCTION_BITMOD = 0x05,
    INSTRUCTION_LOAD_TX0 = 0x40,
    INSTRUCTION_LOAD_TX1 = 0x42,
    INSTRUCTION_LOAD_TX2 = 0x44,
    INSTRUCTION_RTS_TX0 = 0x81,
    INSTRUCTION_RTS_TX1 = 0x82,
    INSTRUCTION_RTS_TX2 = 0x84,
    INSTRUCTION_RTS_ALL = 0x87,
    INSTRUCTION_READ_RX0 = 0x90,
    INSTRUCTION_READ_RX1 = 0x94,
    INSTRUCTION_READ_STATUS = 0xA0,
    INSTRUCTION_RX_STATUS = 0xB0,
    INSTRUCTION_RESET = 0xC0
} INSTRUCTION;

typedef enum
{
    MCP_RXF0SIDH = 0x00,
    MCP_RXF0SIDL = 0x01,
    MCP_RXF0EID8 = 0x02,
    MCP_RXF0EID0 = 0x03,
    MCP_RXF1SIDH = 0x04,
    MCP_RXF1SIDL = 0x05,
    MCP_RXF1EID8 = 0x06,
    MCP_RXF1EID0 = 0x07,
    MCP_RXF2SIDH = 0x08,
    MCP_RXF2SIDL = 0x09,
    MCP_RXF2EID8 = 0x0A,
    MCP_RXF2EID0 = 0x0B,
    MCP_CANSTAT = 0x0E,
    MCP_CANCTRL = 0x0F,
    MCP_RXF3SIDH = 0x10,
    MCP_RXF3SIDL = 0x11,
    MCP_RXF3EID8 = 0x12,
    MCP_RXF3EID0 = 0x13,
    MCP_RXF4SIDH = 0x14,
    MCP_RXF4SIDL = 0x15,
    MCP_RXF4EID8 = 0x16,
    MCP_RXF4EID0 = 0x17,
    MCP_RXF5SIDH = 0x18,
    MCP_RXF5SIDL = 0x19,
    MCP_RXF5EID8 = 0x1A,
    MCP_RXF5EID0 = 0x1B,
    MCP_TEC = 0x1C,
    MCP_REC = 0x1D,
    MCP_RXM0SIDH = 0x20,
    MCP_RXM0SIDL = 0x21,
    MCP_RXM0EID8 = 0x22,
    MCP_RXM0EID0 = 0x23,
    MCP_RXM1SIDH = 0x24,
    MCP_RXM1SIDL = 0x25,
    MCP_RXM1EID8 = 0x26,
    MCP_RXM1EID0 = 0x27,
    MCP_CNF3 = 0x28,
    MCP_CNF2 = 0x29,
    MCP_CNF1 = 0x2A,
    MCP_CANINTE = 0x2B,
    MCP_CANINTF = 0x2C,
    MCP_EFLG = 0x2D,
    MCP_TXB0CTRL = 0x30,
    MCP_TXB0SIDH = 0x31,
    MCP_TXB0SIDL = 0x32,
    MCP_TXB0EID8 = 0x33,
    MCP_TXB0EID0 = 0x34,
    MCP_TXB0DLC = 0x35,
    MCP_TXB0DATA = 0x36,
    MCP_TXB1CTRL = 0x40,
    MCP_TXB1SIDH = 0x41,
    MCP_TXB1SIDL = 0x42,
    MCP_TXB1EID8 = 0x43,
    MCP_TXB1EID0 = 0x44,
    MCP_TXB1DLC = 0x45,
    MCP_TXB1DATA = 0x46,
    MCP_TXB2CTRL = 0x50,
    MCP_TXB2SIDH = 0x51,
    MCP_TXB2SIDL = 0x52,
    MCP_TXB2EID8 = 0x53,
    MCP_TXB2EID0 = 0x54,
    MCP_TXB2DLC = 0x55,
    MCP_TXB2DATA = 0x56,
    MCP_RXB0CTRL = 0x60,
    MCP_RXB0SIDH = 0x61,
    MCP_RXB0SIDL = 0x62,
    MCP_RXB0EID8 = 0x63,
    MCP_RXB0EID0 = 0x64,
    MCP_RXB0DLC = 0x65,
    MCP_RXB0DATA = 0x66,
    MCP_RXB1CTRL = 0x70,
    MCP_RXB1SIDH = 0x71,
    MCP_RXB1SIDL = 0x72,
    MCP_RXB1EID8 = 0x73,
    MCP_RXB1EID0 = 0x74,
    MCP_RXB1DLC = 0x75,
    MCP_RXB1DATA = 0x76
} REGISTER;

static const uint32_t SPI_CLOCK = 10000000; // 10MHz

static const int N_TXBUFFERS = 3;
static const int N_RXBUFFERS = 2;

static const struct TXBn_REGS
{
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
} TXB[N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}};

static const struct RXBn_REGS
{
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
    CANINTF CANINTF_RXnIF;
} RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}};

void startSPI()
{
    SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPICS, LOW);
}

void endSPI()
{
    digitalWrite(SPICS, HIGH);
    SPI.endTransaction();
}

uint8_t mcp2515_read_register(const REGISTER reg)
{
    startSPI();
    SPI.transfer(INSTRUCTION_READ);
    SPI.transfer(reg);
    uint8_t ret = SPI.transfer(0x00);
    endSPI();

    return ret;
}

void mcp2515_read_registers(const REGISTER reg, uint8_t values[], const uint8_t n)
{
    startSPI();
    SPI.transfer(INSTRUCTION_READ);
    SPI.transfer(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i = 0; i < n; i++)
    {
        values[i] = SPI.transfer(0x00);
    }
    endSPI();
}

void mcp2515_set_register(const REGISTER reg, const uint8_t value)
{
    startSPI();
    SPI.transfer(INSTRUCTION_WRITE);
    SPI.transfer(reg);
    SPI.transfer(value);
    endSPI();
}

void mcp2515_set_registers(const REGISTER reg, const uint8_t values[], const uint8_t n)
{
    startSPI();
    SPI.transfer(INSTRUCTION_WRITE);
    SPI.transfer(reg);
    for (uint8_t i = 0; i < n; i++)
    {
        SPI.transfer(values[i]);
    }
    endSPI();
}

void mcp2515_modify_register(const REGISTER reg, const uint8_t mask, const uint8_t data)
{
    startSPI();
    SPI.transfer(INSTRUCTION_BITMOD);
    SPI.transfer(reg);
    SPI.transfer(mask);
    SPI.transfer(data);
    endSPI();
}

uint8_t mcp2515_get_status(void)
{
    startSPI();
    SPI.transfer(INSTRUCTION_READ_STATUS);
    uint8_t i = SPI.transfer(0x00);
    endSPI();

    return i;
}

ERROR mcp2515_set_mode(const CANCTRL_REQOP_MODE mode)
{
    mcp2515_modify_register(MCP_CANCTRL, CANCTRL_REQOP, mode);

    unsigned long endTime = millis() + 10;
    bool modeMatch = false;
    while (millis() < endTime)
    {
        uint8_t newmode = mcp2515_read_register(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch)
        {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;
}

ERROR mcp2515_set_config_mode()
{
    return mcp2515_set_mode(CANCTRL_REQOP_CONFIG);
}

ERROR mcp2515_set_listenonly_mode()
{
    return mcp2515_set_mode(CANCTRL_REQOP_LISTENONLY);
}

ERROR mcp2515_set_sleep_mode()
{
    return mcp2515_set_mode(CANCTRL_REQOP_SLEEP);
}

ERROR mcp2515_set_loopback_mode()
{
    return mcp2515_set_mode(CANCTRL_REQOP_LOOPBACK);
}

ERROR mcp2515_set_normal_mode()
{
    return mcp2515_set_mode(CANCTRL_REQOP_NORMAL);
}

ERROR mcp2515_set_bitrate(const CAN_SPEED canSpeed)
{
    return mcp2515_set_bitrate_(canSpeed, MCP_16MHZ);
}

ERROR mcp2515_set_bitrate_(const CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
    ERROR error = mcp2515_set_config_mode();
    if (error != ERROR_OK)
    {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
    case (MCP_8MHZ):
        switch (canSpeed)
        {
        case (CAN_5KBPS): //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

        case (CAN_10KBPS): //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

        case (CAN_20KBPS): //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

        case (CAN_31K25BPS): //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

        case (CAN_33KBPS): //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            break;
        }
        break;

    case (MCP_16MHZ):
        switch (canSpeed)
        {
        case (CAN_5KBPS): //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

        case (CAN_10KBPS): //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

        case (CAN_20KBPS): //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

        case (CAN_33KBPS): //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

        case (CAN_83K3BPS): //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            break;
        }
        break;

    case (MCP_20MHZ):
        switch (canSpeed)
        {
        case (CAN_33KBPS): //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

        case (CAN_83K3BPS): //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            break;
        }
        break;

    default:
        set = 0;
        break;
    }

    if (set)
    {
        mcp2515_set_register(MCP_CNF1, cfg1);
        mcp2515_set_register(MCP_CNF2, cfg2);
        mcp2515_set_register(MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else
    {
        return ERROR_FAIL;
    }
}

ERROR mcp2515_set_clkout(const CAN_CLKOUT divisor)
{
    if (divisor == CLKOUT_DISABLE)
    {
        /* Turn off CLKEN */
        mcp2515_modify_register(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

        /* Turn on CLKOUT for SOF */
        mcp2515_modify_register(MCP_CNF3, CNF3_SOF, CNF3_SOF);
        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    mcp2515_modify_register(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    mcp2515_modify_register(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    mcp2515_modify_register(MCP_CNF3, CNF3_SOF, 0x00);
    return ERROR_OK;
}

void mcp2515_prepare_id(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext)
    {
        buffer[MCP_EID0] = (uint8_t)(canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t)(canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t)(canid >> 5);
    }
    else
    {
        buffer[MCP_SIDH] = (uint8_t)(canid >> 3);
        buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

ERROR mcp2515_set_filter_mask(const MASK mask, const bool ext, const uint32_t ulData)
{
    ERROR res = mcp2515_set_config_mode();
    if (res != ERROR_OK)
    {
        return res;
    }

    uint8_t tbufdata[4];
    mcp2515_prepare_id(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask)
    {
    case MASK0:
        reg = MCP_RXM0SIDH;
        break;
    case MASK1:
        reg = MCP_RXM1SIDH;
        break;
    default:
        return ERROR_FAIL;
    }

    mcp2515_set_registers(reg, tbufdata, 4);

    return ERROR_OK;
}

ERROR mcp2515_set_filter(const RXF num, const bool ext, const uint32_t ulData)
{
    ERROR res = mcp2515_set_config_mode();
    if (res != ERROR_OK)
    {
        return res;
    }

    REGISTER reg;

    switch (num)
    {
    case RXF0:
        reg = MCP_RXF0SIDH;
        break;
    case RXF1:
        reg = MCP_RXF1SIDH;
        break;
    case RXF2:
        reg = MCP_RXF2SIDH;
        break;
    case RXF3:
        reg = MCP_RXF3SIDH;
        break;
    case RXF4:
        reg = MCP_RXF4SIDH;
        break;
    case RXF5:
        reg = MCP_RXF5SIDH;
        break;
    default:
        return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    mcp2515_prepare_id(tbufdata, ext, ulData);
    mcp2515_set_registers(reg, tbufdata, 4);

    return ERROR_OK;
}

void mcp2515_init()
{
    SPI.begin();
    pinMode(SPICS, OUTPUT);
    endSPI();
}

ERROR mcp2515_reset(void)
{
    startSPI();
    SPI.transfer(INSTRUCTION_RESET);
    endSPI();

    delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    mcp2515_set_registers(MCP_TXB0CTRL, zeros, 14);
    mcp2515_set_registers(MCP_TXB1CTRL, zeros, 14);
    mcp2515_set_registers(MCP_TXB2CTRL, zeros, 14);

    mcp2515_set_register(MCP_RXB0CTRL, 0);
    mcp2515_set_register(MCP_RXB1CTRL, 0);

    mcp2515_set_register(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    mcp2515_modify_register(MCP_RXB0CTRL,
                            RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                            RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    mcp2515_modify_register(MCP_RXB1CTRL,
                            RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                            RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i = 0; i < 6; i++)
    {
        bool ext = (i == 1);
        ERROR result = mcp2515_set_filter(filters[i], ext, 0);
        if (result != ERROR_OK)
        {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i = 0; i < 2; i++)
    {
        ERROR result = mcp2515_set_filter_mask(masks[i], true, 0);
        if (result != ERROR_OK)
        {
            return result;
        }
    }

    return ERROR_OK;
}

ERROR mcp2515_send_message_(const TXBn txbn, const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    mcp2515_prepare_id(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    mcp2515_set_registers(txbuf->SIDH, data, 5 + frame->can_dlc);

    mcp2515_modify_register(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    uint8_t ctrl = mcp2515_read_register(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0)
    {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

ERROR mcp2515_send_message(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAILTX;
    }

    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for (int i = 0; i < N_TXBUFFERS; i++)
    {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = mcp2515_read_register(txbuf->CTRL);
        if ((ctrlval & TXB_TXREQ) == 0)
        {
            return mcp2515_send_message_(txBuffers[i], frame);
        }
    }

    return ERROR_ALLTXBUSY;
}

ERROR mcp2515_read_message_(const RXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    mcp2515_read_registers(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

    if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK)
    {
        id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id << 8) + tbufdata[MCP_EID8];
        id = (id << 8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAIL;
    }

    uint8_t ctrl = mcp2515_read_register(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR)
    {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    mcp2515_read_registers(rxb->DATA, frame->data, dlc);

    mcp2515_modify_register(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;
}

ERROR mcp2515_read_message(struct can_frame *frame)
{
    ERROR rc;
    uint8_t stat = mcp2515_get_status();

    if (stat & STAT_RX0IF)
    {
        rc = mcp2515_read_message_(RXB0, frame);
    }
    else if (stat & STAT_RX1IF)
    {
        rc = mcp2515_read_message_(RXB1, frame);
    }
    else
    {
        rc = ERROR_NOMSG;
    }

    return rc;
}

bool mcp2515_check_receive(void)
{
    uint8_t res = mcp2515_get_status();
    if (res & STAT_RXIF_MASK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool mcp2515_check_error(void)
{
    uint8_t eflg = mcp2515_get_error_flags();

    if (eflg & EFLG_ERRORMASK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t mcp2515_get_error_flags(void)
{
    return mcp2515_read_register(MCP_EFLG);
}

void mcp2515_clear_rxnovrFlags(void)
{
    mcp2515_modify_register(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t getInterrupts(void)
{
    return mcp2515_read_register(MCP_CANINTF);
}

void mcp2515_clear_interrupts(void)
{
    mcp2515_set_register(MCP_CANINTF, 0);
}

uint8_t mcp2515_get_interrupt_mask(void)
{
    return mcp2515_read_register(MCP_CANINTE);
}

void mcp2515_clear_tx_interrupts(void)
{
    mcp2515_modify_register(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void mcp2515_clear_rxnovr(void)
{
    uint8_t eflg = mcp2515_get_error_flags();
    if (eflg != 0)
    {
        mcp2515_clear_rxnovrFlags();
        mcp2515_clear_interrupts();
        //mcp2515_modify_register(MCP_CANINTF, CANINTF_ERRIF, 0);
    }
}

void mcp2515_clear_merr()
{
    //mcp2515_modify_register(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //mcp2515_clear_interrupts();
    mcp2515_modify_register(MCP_CANINTF, CANINTF_MERRF, 0);
}

void mcp2515_clear_errif()
{
    //mcp2515_modify_register(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //mcp2515_clear_interrupts();
    mcp2515_modify_register(MCP_CANINTF, CANINTF_ERRIF, 0);
}

#include "ads1120.h"



void ads_spi_xfer(Ads1120_t* adc, uint8_t* data, int len){
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, 0);
    HAL_SPI_TransmitReceive(adc->hspi, data, data, len, 10);
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, 1);
}


void ads_reset(Ads1120_t* adc){
    uint8_t data = ADS1120_CMD_RESET;
    ads_spi_xfer(adc, &data, 1);
}


void ads_begin(Ads1120_t* adc, uint32_t config){
    adc->init_config = config;

    uint8_t data[5];
    data[0] = ADS1120_CMD_WREG | 3; // starts at register 0
    data[1] = (adc->init_config >> 24) & 0xFF;
    data[2] = (adc->init_config >> 16) & 0xFF;
    data[3] = (adc->init_config >> 8) & 0xFF;
    data[4] = adc->init_config & 0xFF;

    ads_spi_xfer(adc, data, 5);
}


void ads_start_sync(Ads1120_t* adc){
    uint8_t data = ADS1120_CMD_START_SYNC;
    ads_spi_xfer(adc, &data, 1);
}


void ads_set_input_mux(Ads1120_t* adc, uint8_t setting){
    uint8_t buf[2];
    buf[0] = ADS1120_CMD_RREG;
    buf[1] = 0;

    ads_spi_xfer(adc, buf, 2);

    adc->mux_setting = setting;

    buf[1] &= 0x0F; // clear mux bits [7:4]
    buf[1] |= adc->mux_setting;
    buf[0] = ADS1120_CMD_WREG; // rr = 0, nn = 0

    ads_spi_xfer(adc, buf, 2);
}


float ads_get_voltage(Ads1120_t* adc){
    uint8_t buf[3];

    buf[0] = ADS1120_CMD_RDATA;

    ads_spi_xfer(adc, buf, 3);

    int16_t conv = (buf[1] << 8) | buf[2];

    if (conv >= 0){
        return (adc->vref*conv)/ADS1120_FS_COUNTS_POS;
    } else {
        return (adc->vref*conv)/ADS1120_FS_COUNTS_NEG;
    }
}
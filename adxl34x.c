
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "adxl34x.h"

#define BITMASK_SET(x, mask, value) (((x) & ~(mask)) | ((value) & (mask)))
#define BITSHIFT_LEFT(x, n) ((x) = (x) << (n))
#define STANDARD_1G_10Bit           0x100
#define THRESHOLD_1G_10Bit          0x012               // For a accepted deviation of 35 mG and Temperature Drift of 0.8mG/°C at 25°C + Error Margin of 10mG

    // Helper Functions

static uint8_t downgrade10bitTo8bit(uint16_t value){
    float mg_data = (float)value * 3.90625f;
    mg_data = mg_data / 15.6f;
    uint8_t ret_value = (int8_t) mg_data;
    if (ret_value < 0){
        ret_value = ~abs(ret_value) + 1;
    }
    return ret_value;
}
void adxl_BitMask_Edit_Sing_Reg(const adxl34x_t *adxl, uint8_t reg, uint8_t BitsToZero, uint8_t BitsToSet){
    uint8_t regvalue;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(reg), &regvalue, 1);
    regvalue = BITMASK_SET(regvalue, BitsToZero, BitsToSet);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(reg), &regvalue, 1);
}


uint8_t adxl_getDeviceID(const adxl34x_t *adxl){
    uint8_t id;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_DEVID), &id, 1);
    return id;
}

    // BW_RATE
void adxl_LowPowerMode(const adxl34x_t *adxl, bool low_power){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_BW_RATE), &value, 1);
    value = BITMASK_SET(value, 0x10, low_power << 4);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_BW_RATE), &value, 1);
}
void adxl_SamplingRate(const adxl34x_t *adxl, adxl_rate_freq_t s_freq){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_BW_RATE), &value, 1);
    value = BITMASK_SET(value, 0x0F, s_freq);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_BW_RATE), &value, 1);
}
    //POWER_CTL
void adxl_link_Act_Inact_func(const adxl34x_t *adxl, bool ena_link){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
    value = BITMASK_SET(value, 0x20, ena_link << 5);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
}
void adxl_ena_AutoSleep(const adxl34x_t *adxl, bool ena_AutoSleep){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
    value = BITMASK_SET(value, 0x10, ena_AutoSleep << 4);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
}
void adxl_ena_Measurement(const adxl34x_t *adxl, bool ena_Measure){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
    value = BITMASK_SET(value, 0x08, ena_Measure << 3);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
}
void adxl_Sleep(const adxl34x_t *adxl, bool ena_Sleep){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
    value = BITMASK_SET(value, 0x04, ena_Sleep << 2);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
}
void adxl_SleepSampleFreq(const adxl34x_t *adxl, adxl_sleep_rate_freq_t freq){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
    value = BITMASK_SET(value, 0x03, freq);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_POWER_CTL), &value, 1);
}
    //INT_ENABLE
void adxl_enableInterrupts(const adxl34x_t *adxl, adxl_int_conf_t config){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_INT_ENABLE), &value, 1);
    value = BITMASK_SET(value, 0xFF, config.value);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_INT_ENABLE), &value, 1);
}
    
void adxl_disableInterrupts(const adxl34x_t *adxl, adxl_int_conf_t config){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_INT_ENABLE), &value, 1);
    value = BITMASK_SET(value, config.value, 0x00);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_INT_ENABLE), &value, 1);
}
    //INT_MAP
void adxl_mapInterrupts(const adxl34x_t *adxl, adxl_int_conf_t config){
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_INT_MAP), &config, 1);
}
    //INT_SOURCE
adxl_int_src_t adxl_InterruptSrc(const adxl34x_t *adxl){
    adxl_int_src_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_INT_SOURCE), &value, 1);
    return value;
}
void adxl_setRange(const adxl34x_t *adxl, adxl_range_t range){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_DATA_FORMAT), &value, 1);
    value = BITMASK_SET(value, 0x0B, range);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_DATA_FORMAT), &value, 1); 
}


uint16_t adxl_getX(const adxl34x_t *adxl){
    uint16_t x;
    adxl->read(adxl->handle, SPI_READ_MULT_BYTES(ADXL34x_REG_DATAX0), &x, 2);
    return x;
}
uint16_t adxl_getY(const adxl34x_t *adxl){
    uint16_t y;
    adxl->read(adxl->handle, SPI_READ_MULT_BYTES(ADXL34x_REG_DATAY0), &y, 2);
    return y;
}
uint16_t adxl_getZ(const adxl34x_t *adxl){
    uint16_t z;
    adxl->read(adxl->handle, SPI_READ_MULT_BYTES(ADXL34x_REG_DATAZ0), &z, 2);
    return z;
}
adxl_axis_t adxl_get_axis(const adxl34x_t *adxl){
    adxl_axis_t axis;
    adxl->read(adxl->handle, SPI_READ_MULT_BYTES(ADXL34x_REG_DATAX0), &axis, 6);
    return axis;
}
void adxl_FIFO_Mode(const adxl34x_t *adxl, adxl_FIFO_Mode_t mode){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_FIFO_CTL), &value, 1);
    value = BITMASK_SET(value, 0xC0, mode);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_FIFO_CTL), &value, 1);
}
void adxl_FIFO_Samples(const adxl34x_t *adxl, uint8_t n_samples){
    n_samples--;    //The device holds 31 samples in its FIFO + one in the output reg, so 32
    if(n_samples > 31)n_samples = 31;       //It is 32, but the FIFO_CTL, only has 5 bits, so the max is.... 31
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_FIFO_CTL), &value, 1);
    value = BITMASK_SET(value, 0x1F, n_samples);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_FIFO_CTL), &value, 1);
}
void adxl_FIFO_Set_IntPin(const adxl34x_t *adxl, uint8_t pin){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_FIFO_CTL), &value, 1);
    value = BITMASK_SET(value, 0x20, pin ? 0x20 : 0x00);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_FIFO_CTL), &value, 1);
}
uint8_t adxl_FIFO_Triger_event(const adxl34x_t *adxl){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_FIFO_STATUS), &value, 1);
    return FIFO_TRIG(value);
}
uint8_t adxl_FIFO_Entries(const adxl34x_t *adxl){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_FIFO_STATUS), &value, 1);
    return FIFO_ENTRIES(value);
}

void adxl_invert_Int(const adxl34x_t *adxl, bool activeLow){
    uint8_t value;
    adxl->read(adxl->handle, SPI_READ_SING_BYTE(ADXL34x_REG_DATA_FORMAT), &value, 1);
    value = BITMASK_SET(value, 0x20, activeLow << 5);
    adxl->write(adxl->handle, SPI_WRITE_SING_BYTE(ADXL34x_REG_DATA_FORMAT), &value, 1);
}

adxl_err_t adxl_calibrate(const adxl34x_t *adxl, uint8_t n_samples){
    // To calibrate, we take n samples at the specified recommended sampling rate (100 Hz)
    // And acumulate them, then we just divide the acumulated value 
    uint32_t x = 0, y = 0, z = 0;
    if((n_samples % 2) != 0)n_samples++;    //Lets make n_samples base 2, for fcks sake
    if((n_samples > 32))  n_samples = 32;
    adxl_setRange(adxl, _2G);
    adxl_SamplingRate(adxl, _100Hz);
    adxl_FIFO_Mode(adxl, FIFO);
    adxl_FIFO_Samples(adxl, n_samples);     //Set number of samples that the device will take. Max is 32, and I dont think more are needed
    adxl_int_conf_t config;
    config.bits.watermark = 1;
    adxl_enableInterrupts(adxl, config);
    adxl_mapInterrupts(adxl, config);            // Mapping watermark interrupt to interrupt src 1 (2)
    uint32_t int_src = adxl->waitInterrupt(adxl->IntSrc_1, false);     //Now it waits here until, or there is timeout or the interrupt happens
    if((int_src & adxl->IntSrc_1) == 0) return adxl_err_timeout;
    for (int i = 0; i < n_samples; i++){
        adxl_axis_t calibration_buff = adxl_get_axis(adxl);
        x += calibration_buff.X_axis;
        y += calibration_buff.Y_axis;
        z += calibration_buff.Z_axis;
    }
    uint8_t shift = log2(n_samples);
    x = x >> shift;
    y = y >> shift;
    z = z >> shift;
    // Or a division is actually more secure? I mean... when n_samples not in base 2 is...
    // Now we have to write the correction back in the reg: the reg operate with a scale of 15,6 mg/LSB in 2's complement
    // With 10 Bits, 1G, is 0x100, so now we search the one that is affected by one 1G
    if(fabs(STANDARD_1G_10Bit - x) < THRESHOLD_1G_10Bit) x -=STANDARD_1G_10Bit;
    else if(fabs(STANDARD_1G_10Bit - y) < THRESHOLD_1G_10Bit) y -=STANDARD_1G_10Bit;
    else if(fabs(STANDARD_1G_10Bit - z) < THRESHOLD_1G_10Bit) z -=STANDARD_1G_10Bit;
    else{
        return adxl_err_cal_not_valid;
    }
    uint8_t calibration_val[3];
    calibration_val[0] = downgrade10bitTo8bit(x);
    calibration_val[1] = downgrade10bitTo8bit(y);
    calibration_val[2] = downgrade10bitTo8bit(z);
    adxl->write(adxl->handle, SPI_WRITE_MULT_BYTES(ADXL34x_REG_OFSX), calibration_val, 3);
    return adxl_OK;   
}
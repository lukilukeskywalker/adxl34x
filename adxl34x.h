#ifndef __ADXL34x_H__
#define __ADXL34x_H__

typedef struct{
    int32_t (* read)(void *handle, uint8_t reg, void *in_data, size_t in_size);
    int32_t (* write)(void *handle, uint8_t reg, const void *out_data, size_t out_size);
    void *handle;
    int32_t (* waitInterrupt)(uint32_t IntSrcToWait, bool WaitAll);
    int32_t IntSrc_0;
    int32_t IntSrc_1;
}adxl34x_t;

// Register Addresses from the datasheet
#define ADXL34x_REG_DEVID                   0x00
#define ADXL34x_REG_THRESH_TAP              0x1D
#define ADXL34x_REG_OFSX                    0x1E
#define ADXL34x_REG_OFSY                    0x1F
#define ADXL34x_REG_OFSZ                    0x20
#define ADXL34x_REG_DUR                     0x21
#define ADXL34x_REG_Latent                  0x22
#define ADXL34x_REG_Window                  0x23
#define ADXL34x_REG_THRESH_ACT              0x24
#define ADXL34x_REG_THRESH_INACT            0x25
#define ADXL34x_REG_TIME_INACT              0x26
#define ADXL34x_REG_ACT_INACT_CTL           0x27
#define ADXL34x_REG_THRESH_FF               0x28
#define ADXL34x_REG_TIME_FF                 0x29
#define ADXL34x_REG_TAP_AXES                0x2A
#define ADXL34x_REG_ACT_TAP_STATUS          0x2B
#define ADXL34x_REG_BW_RATE                 0x2C
#define ADXL34x_REG_POWER_CTL               0x2D
#define ADXL34x_REG_INT_ENABLE              0x2E
#define ADXL34x_REG_INT_MAP                 0x2F
#define ADXL34x_REG_INT_SOURCE              0x30
#define ADXL34x_REG_DATA_FORMAT             0x31
#define ADXL34x_REG_DATAX0                  0x32
#define ADXL34x_REG_DATAX1                  0x33
#define ADXL34x_REG_DATAY0                  0x34
#define ADXL34x_REG_DATAY1                  0x35
#define ADXL34x_REG_DATAZ0                  0x36
#define ADXL34x_REG_DATAZ1                  0x37
#define ADXL34x_REG_FIFO_CTL                0x38
#define ADXL34x_REG_FIFO_STATUS             0x39

//Specific Registers ADXL344 and ADXL346
#define ADXL34x_REG_TAP_SIGN                0x3A
#define ADXL34x_REG_ORIENT_CONF             0x3B
#define ADXL34x_REG_ORIENT                  0x3C

//FILTER MACROS
#define FIFO_TRIG(x)                        ((x >> 7) & 0x01)
#define FIFO_ENTRIES(x)                     (x & 0x3F)
#define SPI_READ_MULT_BYTES(x)              (x | 0xC0)  //7 Bit indicates Read op, 6 bit indicates Multiple Bytes
#define SPI_READ_SING_BYTE(x)               (x | 0x80)  //7 Bit indicates Read Op
#define SPI_WRITE_MULT_BYTES(x)             (x | 0x40)  //6 Bit indicates Write Op
#define SPI_WRITE_SING_BYTE(x)              (x)


typedef union  {
    uint8_t value;
    struct {                            /*---EXPLANATION---*/
        uint8_t overrun     : 1;        // Interrupt when the FIFO starts to overwrite unread data in the FIFO (In BYPASS Mode), or when the FIFO is full
        uint8_t watermark   : 1;        // Interrupt when n_samples in FIFO equals (or I guess Overpasses) the sample bits in FIFO_CTL
        uint8_t free_fall   : 1;        // Interrupt when acceleration of less than THRESS_FF happens over longer period than TIME_FF (on all axes)
        uint8_t inactivity  : 1;        // Interrupt when acceleration less than THRESH_INACT for longer than the time stored in TIME_INACT (MAX time is 255s (4Min))
        uint8_t activity    : 1;        // Interrupt when acceleration is greater than the value stored in the THRESH_ACT
        uint8_t double_tap  : 1;        // Interrupt when double acceleration events happen over threshold THRESH_TAP, in less than specified DUR reg
        uint8_t single_tap  : 1;        // Interrupt when single acceleration over threshold THRESH_TAP
        uint8_t data_ready  : 1;        // Interrupt when new data is avaliable 
    } bits;
} adxl_int_conf_t;

typedef adxl_int_conf_t adxl_int_src_t;

typedef enum{
    _10cHz,
    _20cHz,
    _39cHz,
    _78cHz,
    _156cHz,
    _313cHz,
    _625cHz,
    _1250cHz,
    _25Hz,
    _50Hz,
    _100Hz,
    _200Hz,
    _400Hz,
    _800Hz,
    _1600Hz,
    _3200Hz,
} adxl_rate_freq_t;
typedef enum{
    BYPASS  = 0x00,
    FIFO    = 0x40,
    STREAM  = 0x80,
    TRIGGER = 0xD0,
}adxl_FIFO_Mode_t;
typedef enum{
    _2G     = 0x00,
    _4G     = 0x01,
    _8G     = 0x02,
    _16G    = 0x03,
    _FULL_RES = 0x08,
}adxl_range_t;

typedef struct {
    int16_t X_axis;
    int16_t Y_axis;
    int16_t Z_axis;
} adxl_axis_t;

typedef enum {
    adxl_OK = 0x00,
    adxl_err,
    adxl_err_timeout,
    adxl_err_cal_not_valid,

} adxl_err_t;

typedef enum {
    _8Hz,
    _4Hz,
    _2Hz,
    _1Hz,
}adxl_sleep_rate_freq_t;

uint8_t adxl_getDeviceID(const adxl34x_t *adxl);

void adxl_LowPowerMode(const adxl34x_t *adxl, bool low_power);
void adxl_SamplingRate(const adxl34x_t *adxl, adxl_rate_freq_t s_freq);
void adxl_link_Act_Inact_func(const adxl34x_t *adxl, bool ena_link);
void adxl_ena_AutoSleep(const adxl34x_t *adxl, bool ena_AutoSleep);
void adxl_ena_Measurement(const adxl34x_t *adxl, bool ena_Measure);
void adxl_Sleep(const adxl34x_t *adxl, bool ena_Sleep);
void adxl_SleepSampleFreq(const adxl34x_t *adxl, adxl_sleep_rate_freq_t freq);

void adxl_enableInterrupts(const adxl34x_t *adxl, adxl_int_conf_t config);
void adxl_disableInterrupts(const adxl34x_t *adxl, adxl_int_conf_t config);
void adxl_mapInterrupts(const adxl34x_t *adxl, adxl_int_conf_t config);
adxl_int_src_t adxl_InterruptSrc(const adxl34x_t *adxl);
void adxl_setRange(const adxl34x_t *adxl, adxl_range_t range);
int16_t adxl_getX(const adxl34x_t *adxl);
int16_t adxl_getY(const adxl34x_t *adxl);
int16_t adxl_getZ(const adxl34x_t *adxl);
adxl_axis_t adxl_get_axis(const adxl34x_t *adxl);
void adxl_FIFO_Mode(const adxl34x_t *adxl, adxl_FIFO_Mode_t mode);
void adxl_FIFO_Samples(const adxl34x_t *adxl, uint8_t n_samples);
void adxl_FIFO_Set_IntPin(const adxl34x_t *adxl, uint8_t pin);
uint8_t adxl_FIFO_Triger_event(const adxl34x_t *adxl);
uint8_t adxl_FIFO_Entries(const adxl34x_t *adxl);
void adxl_invert_Int(const adxl34x_t *adxl, bool activeLow);
adxl_err_t adxl_calibrate(const adxl34x_t *adxl, uint8_t n_samples);
adxl_err_t adxl_reset(const adxl34x_t *adxl);





#endif
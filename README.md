# ADXL34x Library

This library provides a set of functions to interact with the ADXL34x accelerometer in a hardware agnostic manner. The library includes functions for initialization, calibration, data retrieval, and interrupt setup.

## Table of Contents
- [Introduction](#introduction)
- [Getting Started](#getting-started)
  - [Initialization](#initialization)
  - [Calibration](#calibration)
  - [Data Retrieval](#data-retrieval)
  - [Interrupt Setup](#interrupt-setup)
  - [FIFO Configuration](#fifo-configuration)
  - [Range Configuration](#range-configuration)
  - [Sample Rate Configuration](#sample-rate-configuration)
- [Contributing](#contributing)
- [License](#license)

## Introduction
The ADXL34x library allows communication with the ADXL34x accelerometer in a hardware agnostic manner. The library supports features such as device initialization, calibration, data retrieval, and interrupt setup.

## Getting Started

### Initialization
To use the ADXL34x library, include the `adxl34x.h` header file in your project. Then, create an instance of the `adxl34x_t` structure, which represents the ADXL34x device. Initialize the handle with your SPI communication functions and the desired interrupt pins.

```c
adxl34x_t adxl = {
    .read  = (void *) spi_read_reg,
    .write = (void *) spi_write_reg,
    .waitInterrupt = (uint32_t *) waitInterrupt,
    .IntSrc_0 = INT_SRC_FLAG_0,
    .IntSrc_1 = INT_SRC_FLAG_1,
    .handle = spi_handle_ptr,
};

// Reset the ADXL34x device
adxl_reset(&adxl);

// Get the device ID
uint8_t chip = adxl_getDeviceID(&adxl);
```

### Calibration
Calibration is performed to compensate for device offsets. The library provides a calibration function that accumulates accelerometer readings and adjusts the device's offset accordingly.

```c
// Calibrate the ADXL34x device
adxl_calibrate(&adxl, 32);
```

### Data Retrieval
Retrieve accelerometer data using functions such as `adxl_getX`, `adxl_getY`, `adxl_getZ`, or `adxl_get_axis`.

```c
int16_t x = adxl_getX(&adxl);
int16_t y = adxl_getY(&adxl);
int16_t z = adxl_getZ(&adxl);

adxl_axis_t axis = adxl_get_axis(&adxl);
```

### Interrupt Setup
Enable, disable, or map interrupts based on your requirements using functions like `adxl_enableInterrupts`, `adxl_disableInterrupts`, `adxl_mapInterrupts`, and `adxl_InterruptSrc`.
Below are the different interrupt modes supported by the accelerometer, as specified in the `adxl_int_conf_t` structure:

```c
typedef union  {
    uint8_t value;
    struct {                            
        uint8_t overrun     : 1;        // Interrupt when the FIFO starts to overwrite unread data in the FIFO (In BYPASS Mode), or when the FIFO is full
        uint8_t watermark   : 1;        // Interrupt when n_samples in FIFO equals (or I guess Overpasses) the sample bits in FIFO_CTL
        uint8_t free_fall   : 1;        // Interrupt when acceleration of less than THRESS_FF happens over a longer period than TIME_FF (on all axes)
        uint8_t inactivity  : 1;        // Interrupt when acceleration less than THRESH_INACT for longer than the time stored in TIME_INACT (MAX time is 255s or 4 minutes)
        uint8_t activity    : 1;        // Interrupt when acceleration is greater than the value stored in THRESH_ACT
        uint8_t double_tap  : 1;        // Interrupt when double acceleration events happen over the threshold THRESH_TAP, in less than the specified DUR reg
        uint8_t single_tap  : 1;        // Interrupt when single acceleration over the threshold THRESH_TAP
        uint8_t data_ready  : 1;        // Interrupt when new data is available 
    } bits;
} adxl_int_conf_t;
```

Here's a brief explanation of each interrupt mode:

- **overrun**: Interrupt triggered when the FIFO starts to overwrite unread data in the FIFO (in BYPASS Mode), or when the FIFO is full.

- **watermark**: Interrupt triggered when the number of samples in the FIFO equals or overpasses the sample bits specified in FIFO_CTL.

- **free_fall**: Interrupt triggered when acceleration is less than THRESS_FF over a longer period than TIME_FF on all axes.

- **inactivity**: Interrupt triggered when acceleration is less than THRESH_INACT for longer than the time stored in TIME_INACT. The maximum time is 255 seconds (4 minutes).

- **activity**: Interrupt triggered when acceleration is greater than the value stored in THRESH_ACT.

- **double_tap**: Interrupt triggered when double acceleration events happen over the threshold THRESH_TAP in less than the specified DUR reg.

- **single_tap**: Interrupt triggered when a single acceleration event occurs over the threshold THRESH_TAP.

- **data_ready**: Interrupt triggered when new data is available.

These interrupt modes provide flexibility in configuring the behavior of the accelerometer based on specific motion or data conditions. Depending on your application requirements, you can enable or disable these interrupt modes to tailor the response of the accelerometer to different scenarios.

```c
adxl_int_conf_t config;
config.bits.watermark = 1;

// Enable interrupts based on the configuration
adxl_enableInterrupts(&adxl, config);

// Map interrupts to a specific configuration
adxl_mapInterrupts(&adxl, config);

// Get the interrupt source
adxl_int_src_t int_src = adxl_InterruptSrc(&adxl);
```

### FIFO Configuration:

The accelerometer provides a First In, First Out (FIFO) buffer to store sampled data. You can configure the FIFO in different modes:

- **BYPASS Mode (Default):** In this mode, the FIFO is not used, and data is directly available through the data registers.

- **FIFO Mode:** The FIFO accumulates samples up to a specified threshold, and then triggers an interrupt or stores data for retrieval.

- **Stream Mode:** In this mode, the FIFO continuously streams data, overwriting the oldest data when it reaches the FIFO capacity.

- **Trigger Mode:** The FIFO collects samples when triggered by a specific event, such as a threshold crossing.

In the provided ADXL34x library, you can set the FIFO mode using the `adxl_FIFO_Mode` function:

```c
// Example: Enable FIFO in Stream Mode
adxl_FIFO_Mode(&adxl, STREAM);
```

### Range Configuration:

The accelerometer supports different measurement ranges, representing the maximum acceleration that can be measured along each axis. The available ranges are:

- ±2g
- ±4g
- ±8g
- ±16g

You can set the range using the `adxl_setRange` function:

```c
// Example: Set the measurement range to ±8g
adxl_setRange(&adxl, _8G);
```

### Sample Rate Configuration:

The accelerometer allows you to configure the data output rate, which determines how often new data is sampled and available for reading. The available sample rates are:

- 10 Hz
- 20 Hz
- 39 Hz
- 78 Hz
- 156 Hz
- 313 Hz
- 625 Hz
- 1250 Hz
- 25 Hz
- 50 Hz
- 100 Hz
- 200 Hz
- 400 Hz
- 800 Hz
- 1600 Hz
- 3200 Hz

You can set the sample rate using the `adxl_SamplingRate` function:

```c
// Example: Set the sample rate to 100 Hz
adxl_SamplingRate(&adxl, _100Hz);
```

Adjust the configurations based on your specific application requirements. Keep in mind the trade-offs between resolution, data output rate, and power consumption when choosing these configurations.
## Contributing
Feel free to contribute to the development of this library by opening issues, submitting pull requests, or suggesting improvements.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

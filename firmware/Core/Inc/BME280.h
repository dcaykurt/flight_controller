



#ifndef _BME280_H_
#define _BME280_H_

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "i2c.h"
#include "string.h"
#include "math.h"
#include <stdint.h>
#include <stdbool.h>


#define BME280_ADDRESS 						0x77  // Primary I2C Address  //--> 0x77          
#define BME280_ADDRESS_ALTERNATE  0x76  // Alternate Address    //--> 0x76

#define BMP280_CHIP_ID  0x58 /* BMP280 has chip-id 0x58 */
#define BME280_CHIP_ID  0x60 /* BME280 has chip-id 0x60 */


typedef enum {
    BME280_MODE_SLEEP = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_NORMAL = 3
} BME280_Mode;

typedef enum {
    BME280_FILTER_OFF = 0,
    BME280_FILTER_2 = 1,
    BME280_FILTER_4 = 2,
    BME280_FILTER_8 = 3,
    BME280_FILTER_16 = 4
} BME280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BME280_SKIPPED = 0,          /* no measurement  */
    BME280_ULTRA_LOW_POWER = 1,  /* oversampling x1 */
    BME280_LOW_POWER = 2,        /* oversampling x2 */
    BME280_STANDARD = 3,         /* oversampling x4 */
    BME280_HIGH_RES = 4,         /* oversampling x8 */
    BME280_ULTRA_HIGH_RES = 5    /* oversampling x16 */
} BME280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BME280_STANDBY_05 = 0,      /* stand by time 0.5ms */
    BME280_STANDBY_62 = 1,      /* stand by time 62.5ms */
    BME280_STANDBY_125 = 2,     /* stand by time 125ms */
    BME280_STANDBY_250 = 3,     /* stand by time 250ms */
    BME280_STANDBY_500 = 4,     /* stand by time 500ms */
    BME280_STANDBY_1000 = 5,    /* stand by time 1s */
    BME280_STANDBY_2000 = 6,    /* stand by time 2s BMP280, 10ms BME280 */
    BME280_STANDBY_4000 = 7,    /* stand by time 4s BMP280, 20ms BME280 */
} BME280_StandbyTime;



typedef struct {
    BME280_Mode mode;
    BME280_Filter filter;
    BME280_Oversampling oversampling_pressure;
    BME280_Oversampling oversampling_temperature;
    BME280_Oversampling oversampling_humidity;
    BME280_StandbyTime standby;
} bme280_params_t;


typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    uint16_t addr;

    I2C_HandleTypeDef* i2c;
		I2C_HandleTypeDef* i2c1;

    bme280_params_t params;

    uint8_t  id;        /* Chip ID */
		
		

} BME280_HandleTypedef;




//void HUMA_BNO055_Baslat(I2C_HandleTypeDef a);  HI2C YI DEGISKEN OLARAK ÇEK
//void HUMA_BNO055_Baslat(void);

/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: NORAML
 *      filter: OFF
 *      oversampling: x4
 *      standby time: 250ms
 */
void bme280_init_default_params(bme280_params_t *params);

/**
 * Initialize BMP280 module, probes for the device, soft resets the device,
 * reads the calibration constants, and configures the device using the supplied
 * parameters. Returns true on success otherwise false.
 *
 * The I2C address is assumed to have been initialized in the dev, and
 * may be either BMP280_I2C_ADDRESS_0 or BMP280_I2C_ADDRESS_1. If the I2C
 * address is unknown then try initializing each in turn.
 *
 * This may be called again to soft reset the device and initialize it again.
 */
bool bme280_init(BME280_HandleTypedef *dev, bme280_params_t *params);

/**
 * Start measurement in forced mode.
 * The module remains in forced mode after this call.
 * Do not call this method in normal mode.
 */
bool bme280_force_measurement(BME280_HandleTypedef *dev);

/**
 * Check if BMP280 is busy with measuring temperature/pressure.
 * Return true if BMP280 is busy.
 */
bool bme280_is_measuring(BME280_HandleTypedef *dev);

/**
 * Read compensated temperature and pressure data:
 *
 *  Temperature in degrees Celsius times 100.
 *
 *  Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
 *
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity as a fixed point 22 bit interger and 10 bit fraction format.
 */
bool bme280_read_fixed(BME280_HandleTypedef *dev, int32_t *temperature,
                       uint32_t *pressure, uint32_t *humidity);

/**
 * Read compensated temperature and pressure data:
 *  Temperature in degrees Celsius.
 *  Pressure in Pascals.
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity.
 */
bool bme280_read_float(BME280_HandleTypedef *dev, float *temperature,
                       float *pressure, float *humidity);

void BME280_Init(void);
float BME280_Altitude(void);
void hesap();


#endif  // __BMP280_H__

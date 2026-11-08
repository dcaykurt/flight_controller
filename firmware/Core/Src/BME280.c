

#include "BME280.h"

#include "stm32f4xx_hal.h"
#include "math.h"
#include "gpio.h"
#include "i2c.h"
#include "string.h"
#include "KalmanBME280.h"
//#include "Ringbuf.h"


#define KF_ADAPT			1.0f


#define BME280_REGISTER_DIG_T1  0x88
#define BME280_REGISTER_DIG_T2  0x8A
#define BME280_REGISTER_DIG_T3  0x8C

#define BME280_REGISTER_DIG_P1  0x8E
#define BME280_REGISTER_DIG_P2  0x90
#define BME280_REGISTER_DIG_P3  0x92
#define BME280_REGISTER_DIG_P4  0x94
#define BME280_REGISTER_DIG_P5  0x96
#define BME280_REGISTER_DIG_P6  0x98
#define BME280_REGISTER_DIG_P7  0x9A
#define BME280_REGISTER_DIG_P8  0x9C
#define BME280_REGISTER_DIG_P9  0x9E

#define BME280_REGISTER_DIG_H1  0xA1
#define BME280_REGISTER_DIG_H2  0xE1
#define BME280_REGISTER_DIG_H3  0xE3
#define BME280_REGISTER_DIG_H4  0xE4
#define BME280_REGISTER_DIG_H5  0xE5
#define BME280_REGISTER_DIG_H6  0xE7



#define BME280_REGISTER_CHIPID     0xD0
#define BME280_REGISTER_VERSION    0xD1
#define BME280_REGISTER_SOFTRESET  0xE0
#define BMP280_RESET_VALUE 0xB6



#define BME280_REGISTER_CAL26  0xE1 // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID  0xF2
#define BME280_REGISTER_STATUS  0XF3
#define BME280_REGISTER_CONTROL  0xF4
#define BME280_REGISTER_CONFIG  0xF5
#define BME280_REGISTER_PRESSUREDATA  0xF7
#define BME280_REGISTER_TEMPDATA  0xFA
#define BME280_REGISTER_HUMIDDATA  0xFD


BME280_HandleTypedef bme280;
float pressure, temperature, humidity;
//volatile float KFAltitudeCm, KFClimbrateCps;
float BME280_KFAltitudeCm, BME280_KFClimbrateCps;
float BME280_zAccelAverage;
	

void bme280_init_default_params(bme280_params_t *params) {
	params->mode = BME280_MODE_NORMAL;
	params->filter = BME280_FILTER_16;//8
	params->oversampling_pressure = BME280_ULTRA_HIGH_RES;
	params->oversampling_temperature = BME280_LOW_POWER;
	params->oversampling_humidity = BME280_ULTRA_LOW_POWER;
	params->standby = BME280_STANDBY_05;//250
}

static bool read_register16(BME280_HandleTypedef *dev, uint8_t addr, uint16_t *value) {
	uint16_t tx_buff;
	uint8_t rx_buff[2];
	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, rx_buff, 2, 5000)
			== HAL_OK) {
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return true;
	} else
		return false;

}

static inline int read_data(BME280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
		uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
	//if (HAL_I2C_Mem_Read_DMA(dev->i2c, tx_buff, addr, 1, value, len) == HAL_OK)
		return 0;
	else
		return 1;
}

static inline int read_dataDMA(BME280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
		uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
	//if (HAL_I2C_Mem_Read_DMA(dev->i2c, tx_buff, addr, 1, value, len) == HAL_OK)
		return 0;
	else
		return 1;
}


static bool read_calibration_data(BME280_HandleTypedef *dev) {

	if (read_register16(dev, 0x88, &dev->dig_T1)
			&& read_register16(dev, 0x8a, (uint16_t *) &dev->dig_T2)
			&& read_register16(dev, 0x8c, (uint16_t *) &dev->dig_T3)
			&& read_register16(dev, 0x8e, &dev->dig_P1)
			&& read_register16(dev, 0x90, (uint16_t *) &dev->dig_P2)
			&& read_register16(dev, 0x92, (uint16_t *) &dev->dig_P3)
			&& read_register16(dev, 0x94, (uint16_t *) &dev->dig_P4)
			&& read_register16(dev, 0x96, (uint16_t *) &dev->dig_P5)
			&& read_register16(dev, 0x98, (uint16_t *) &dev->dig_P6)
			&& read_register16(dev, 0x9a, (uint16_t *) &dev->dig_P7)
			&& read_register16(dev, 0x9c, (uint16_t *) &dev->dig_P8)
			&& read_register16(dev, 0x9e, (uint16_t *) &dev->dig_P9)
			&& read_register16(dev, 0xA1, (uint16_t *) &dev->dig_H1)
			&& read_register16(dev, 0xE1, (uint16_t *) &dev->dig_H2)
			&& read_register16(dev, 0xE3, (uint16_t *) &dev->dig_H3)
			&& read_register16(dev, 0xE4, (uint16_t *) &dev->dig_H4)
			&& read_register16(dev, 0xE5, (uint16_t *) &dev->dig_H5)
			&& read_register16(dev, 0xE7, (uint16_t *) &dev->dig_H6)) 
	{

		return true;
	}

	return false;
}

static bool read_hum_calibration_data(BME280_HandleTypedef *dev) {
	uint16_t h4, h5;

	if (!read_data(dev, 0xa1, &dev->dig_H1, 1)
			&& read_register16(dev, 0xe1, (uint16_t *) &dev->dig_H2)
			&& !read_data(dev, 0xe3, &dev->dig_H3, 1)
			&& read_register16(dev, 0xe4, &h4)
			&& read_register16(dev, 0xe5, &h5)
			&& !read_data(dev, 0xe7, (uint8_t *) &dev->dig_H6, 1)) {
		dev->dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
		dev->dig_H5 = h5 >> 4;

		return true;
	}

	return false;
}

static int write_register8(BME280_HandleTypedef *dev, uint8_t addr, uint8_t value) {
	uint16_t tx_buff;

	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Write(dev->i2c, tx_buff, addr, 1, &value, 1, 10000) == HAL_OK)
		return false;
	else
		return true;
}

bool bme280_init(BME280_HandleTypedef *dev, bme280_params_t *params) {

	if (dev->addr != BME280_ADDRESS
			&& dev->addr != BME280_ADDRESS_ALTERNATE) {

		return false;
	}

	if (read_data(dev, BME280_REGISTER_CHIPID, &dev->id, 1)) {
		return false;
	}

	if (dev->id != BME280_CHIP_ID && dev->id != BMP280_CHIP_ID) {

		return false;
	}

	// Soft reset.
	if (write_register8(dev, BME280_REGISTER_SOFTRESET, BMP280_RESET_VALUE)) {
		return false;
	}

	// Wait until finished copying over the NVP data.
	while (1) {
		uint8_t status;
		if (!read_data(dev, BME280_REGISTER_STATUS, &status, 1)
				&& (status & 1) == 0)
			break;
	}

	if (!read_calibration_data(dev)) {
		return false;
	}

	if (dev->id == BME280_CHIP_ID && !read_hum_calibration_data(dev)) {
		return false;
	}

	uint8_t config = (params->standby << 5) | (params->filter << 2);
	if (write_register8(dev, BME280_REGISTER_CONFIG, config)) {
		return false;
	}

	if (params->mode == BME280_MODE_FORCED) {
		params->mode = BME280_MODE_SLEEP;  // initial mode for forced is sleep
	}

	uint8_t ctrl = (params->oversampling_temperature << 5)
			| (params->oversampling_pressure << 2) | (params->mode);

	if (dev->id == BME280_CHIP_ID) {
		// Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
		uint8_t ctrl_hum = params->oversampling_humidity;
		if (write_register8(dev, BME280_REGISTER_CONTROLHUMID, ctrl_hum)) {
			return false;
		}
	}

	if (write_register8(dev, BME280_REGISTER_CONTROL, ctrl)) {
		return false;
	}

	return true;
}

bool bme280_force_measurement(BME280_HandleTypedef *dev) {
	uint8_t ctrl;
	if (read_data(dev, BME280_REGISTER_CONTROL, &ctrl, 1))
		return false;
	ctrl &= ~0xB11;  // clear two lower bits
	ctrl |= BME280_MODE_FORCED;
	if (write_register8(dev, BME280_REGISTER_CONTROL, ctrl)) {
		return false;
	}
	return true;
}

bool bme280_is_measuring(BME280_HandleTypedef *dev) {
	uint8_t status;
	if (read_data(dev, BME280_REGISTER_STATUS, &status, 1))
		return false;
	if (status & (1 << 3)) {
		return true;
	}
	return false;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(BME280_HandleTypedef *dev, int32_t adc_temp,
		int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) dev->dig_T1 << 1)))
			* (int32_t) dev->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) dev->dig_T1)
			* ((adc_temp >> 4) - (int32_t) dev->dig_T1)) >> 12)
			* (int32_t) dev->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(BME280_HandleTypedef *dev, int32_t adc_press,
		int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
	return p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_humidity(BME280_HandleTypedef *dev, int32_t adc_hum,
		int32_t fine_temp) {
	int32_t v_x1_u32r;

	v_x1_u32r = fine_temp - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) dev->dig_H4 << 20)
			- ((int32_t) dev->dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
			* (((((((v_x1_u32r * (int32_t) dev->dig_H6) >> 10)
					* (((v_x1_u32r * (int32_t) dev->dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) dev->dig_H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* (int32_t) dev->dig_H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
	return v_x1_u32r >> 12;
}

bool bme280_read_fixed(BME280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure,
		uint32_t *humidity) {
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[8];

	// Only the BME280 supports reading the humidity.
	if (dev->id != BME280_REGISTER_CHIPID) {
		if (humidity)
			*humidity = 0;
		humidity = NULL;
	}

	// Need to read in one sequence to ensure they match.
	size_t size = humidity ? 8 : 6;
	//if (read_data(dev, 0xf7, data, size)) {
	if (read_dataDMA(dev, 0xf7, data, size)) {	
		return false;
	}

	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	int32_t fine_temp;
	*temperature = compensate_temperature(dev, adc_temp, &fine_temp);
	*pressure = compensate_pressure(dev, adc_pressure, fine_temp);

	if (humidity) {
		int32_t adc_humidity = data[6] << 8 | data[7];
		*humidity = compensate_humidity(dev, adc_humidity, fine_temp);
	}

	return true;
}

bool bme280_read_float(BME280_HandleTypedef *dev, float *temperature, float *pressure,
		float *humidity) {
	int32_t fixed_temperature;
	uint32_t fixed_pressure;
	uint32_t fixed_humidity;
	if (bme280_read_fixed(dev, &fixed_temperature, &fixed_pressure,
			humidity ? &fixed_humidity : NULL)) {
		*temperature = (float) fixed_temperature / 100;
		*pressure = (float) fixed_pressure / 256;
		if (humidity)
			*humidity = (float) fixed_humidity / 1024;
		return true;
	}

	return false;
}

void BME280_Init(void)
{
	
	bme280_init_default_params(&bme280.params);
	bme280.addr = BME280_ADDRESS;
	//bme280.i2c = &hi2c3;

	while (!bme280_init(&bme280, &bme280.params)) {
		HAL_Delay(2000);
	}
	bool bme280p = bme280.id == BMP280_CHIP_ID;
	HAL_Delay(1000);
//Ilk okuma
	HAL_Delay(100);
	while (!bme280_read_float(&bme280, &temperature, &pressure, &humidity)) 
	{
		HAL_Delay(2000);
	}
	
	kalmanFilter4d_configure(1000.0f*400,KF_ADAPT,0, 0.0f, 0.0f);
}

float BME280_FirstPress;
float BME280_Press;
float BME280_temp;
float BME280_alt;
float BME280_CalibBasinc1;
int   BME280_sayac;
float BME280_KalmanResult;

float KFAltitudeCm,KFClimbrateCps;
float zAccelAverage;

float simple_firstpress,simple_press,simple_altitude;

float deneme_press;

float BME280_Altitude()
{	
	bme280_read_float(&bme280, &temperature, &pressure, &humidity);
	deneme_press= pressure/100;
		
	if (BME280_sayac<200)
	{
		BME280_CalibBasinc1 += pressure/100;
		BME280_sayac++;
		BME280_FirstPress = BME280_CalibBasinc1/200;
		simple_firstpress = BME280_CalibBasinc1/200;
	}
	
	if(BME280_sayac>=200)
	{
		BME280_temp = temperature;
		BME280_Press = pressure/100;
		simple_press = pressure/100;

		hesap();

	}
	
	return BME280_KalmanResult;
}

float zAccelAverage,last_zAccelAverage,fark_zAccelAverage;
float RefPress,RefPress_buffer[10];
int RefPress_buffer_count;

float Press,Press_buffer[10];
int Press_buffer_count;

int hareketsiz_sayac;
int hareketli_sayac;

float toplam_altitude,ara_altitude;

float press_100;

void hesap()
{
		//zAccelAverage = ringbuf_averageNewestSamples(10);	
		fark_zAccelAverage = zAccelAverage - last_zAccelAverage;
		last_zAccelAverage=zAccelAverage;
		
		if (fark_zAccelAverage>=-0.0025 && fark_zAccelAverage<=0.0025)//Hareketsiz
		{
			if(hareketsiz_sayac==24)
			{
				for(int i=0;i<10;i++)
				{
					Press += Press_buffer[i];
					Press_buffer[i]=0;
				}
				if(Press!=0)BME280_FirstPress = Press/Press_buffer_count;
				if (RefPress!=0) toplam_altitude= toplam_altitude + 44330 * (1.0 - pow((Press/Press_buffer_count) / (RefPress/RefPress_buffer_count)  , 0.1903));
				RefPress=0;
				RefPress_buffer_count=0;
			}
			if (hareketsiz_sayac<25)
			{
				hareketsiz_sayac++;
			}
			else
			{
				hareketli_sayac=0;
				BME280_FirstPress = pressure/100;
				RefPress_buffer_count++;
				if (RefPress_buffer_count>10) RefPress_buffer_count=10;
				for(int i=0;i<10;i++)
				{
					RefPress_buffer[i]=RefPress_buffer[i+1];
				}
				RefPress_buffer[9]=pressure/100;
				BME280_alt=0.0f;
			}
		}
		
		if (fark_zAccelAverage<-0.0025 || fark_zAccelAverage>0.0025)//Hareketli
		{
			if(hareketli_sayac==24)
			{

				Press=0;
				Press_buffer_count=0;
				for(int i=0;i<10;i++)
				{
					RefPress += RefPress_buffer[i];
					RefPress_buffer[i]=0;
				}
			}
			if (hareketli_sayac<25)
			{
				hareketli_sayac++;
			}
			else
			{
				hareketsiz_sayac=0;
				BME280_Press = pressure/100;				
				Press_buffer_count++;
				if(Press_buffer_count>10) Press_buffer_count=10;
				for(int i=0;i<10;i++)
				{
					Press_buffer[i]=Press_buffer[i+1];
				}
				Press_buffer[9]=BME280_Press;
			}
		}
	
		if(BME280_FirstPress!=0){
	BME280_alt = 44330 * (1.0 - pow((pressure/100) / BME280_FirstPress , 0.1903));
	press_100 = pressure/100;

	ara_altitude = toplam_altitude + BME280_alt;
		
	simple_altitude	= 44330 * (1.0 - pow(simple_press / simple_firstpress , 0.1903));
	kalmanFilter4d_predict(0.04);//0.001//0.1//0.055
	BME280_KalmanResult = kalmanFilter4d_update(ara_altitude,0,(float*)&KFAltitudeCm, (float*)&KFClimbrateCps);//KFAltitudeCm
	
	}
}



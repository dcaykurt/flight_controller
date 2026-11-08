/*
 * HUMA_BNO055 — Bosch BNO055 9 eksenli IMU için minimal I2C sürücüsü
 * HUMA AR-GE, 03.04.2021'de yazılmaya başlandı; LOTUS (2022) ile geliştirildi.
 *
 * Sensör NDOF füzyon modunda çalıştırılır ve Euler açıları
 * (heading/roll/pitch) 1/16 derece çözünürlükle okunur.
 */
#ifndef _HUMA_IHA_BNO055_H_
#define _HUMA_IHA_BNO055_H_

#include "stm32f4xx_hal.h"

void  HUMA_BNO055_Baslat(void);

/* Tek tek register okuyarak (bloklayan) Euler açısı okuma */
float HUMA_BNO055_EULER_ROLL(void);
float HUMA_BNO055_EULER_PITCH(void);
float HUMA_BNO055_EULER_YAW(void);

/* Tüm Euler açılarını tek seferde DMA ile okur.
 * Sonuçlar HUMA_*_Get() fonksiyonlarıyla alınır. */
void  HUMA_BNO055_ALL_READ(void);
float HUMA_Roll_Get(void);
float HUMA_Pitch_Get(void);
float HUMA_Yaw_Get(void);

#endif

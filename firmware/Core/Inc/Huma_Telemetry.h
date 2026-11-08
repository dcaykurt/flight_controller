/*
 * Huma_Telemetry — yer istasyonundan UART ile PID katsayısı alma
 *
 * Paket biçimi:  K<kp>L<ki>M<kd>N     örnek: "K1.65L0.35M0.5N"
 */
#ifndef _HUMA_TELEMETRY_H_
#define _HUMA_TELEMETRY_H_

#include <stdint.h>

void    veri_ayir(char rx_buffer[50]);
float   Read_Kp(void);
float   Read_Ki(void);
float   Read_Kd(void);
uint8_t Read_Loop(void);

#endif

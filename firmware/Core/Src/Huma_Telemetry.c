/*
 * Huma_Telemetry — yer istasyonundan UART ile PID katsayısı alma
 */
#include "Huma_Telemetry.h"
#include <string.h>
#include <stdlib.h>

#define PAKET_BOYU 50

static float   Tel_Kp, Tel_Ki, Tel_Kd;
static uint8_t loop;

/* "K<kp>L<ki>M<kd>N" paketini ayrıştırır; "ABC" komutu loop bayrağını kurar */
void veri_ayir(char rx_buffer[PAKET_BOYU])
{
  char P_s[PAKET_BOYU] = {0}, I_s[PAKET_BOYU] = {0}, D_s[PAKET_BOYU] = {0};
  int  pos_P = 0, pos_I = 0, pos_D = 0, pos_S = 0;
  int  uzunluk = strnlen(rx_buffer, PAKET_BOYU);

  if (rx_buffer[0] == 'K') {
    for (int i = 0; i < uzunluk; i++) {
      if (rx_buffer[i] == 'K') pos_P = i;
      if (rx_buffer[i] == 'L') pos_I = i;
      if (rx_buffer[i] == 'M') pos_D = i;
      if (rx_buffer[i] == 'N') pos_S = i;
    }
    for (int i = 0; i < uzunluk; i++) {
      if (i > pos_P && i < pos_I) P_s[i - pos_P - 1] = rx_buffer[i];
      if (i > pos_I && i < pos_D) I_s[i - pos_I - 1] = rx_buffer[i];
      if (i > pos_D && i < pos_S) D_s[i - pos_D - 1] = rx_buffer[i];
    }
    Tel_Kp = atof(P_s);
    Tel_Ki = atof(I_s);
    Tel_Kd = atof(D_s);
  }

  if (strncmp(rx_buffer, "ABC", 3) == 0) {
    loop = 1;
  }
}

float   Read_Kp(void)   { return Tel_Kp; }
float   Read_Ki(void)   { return Tel_Ki; }
float   Read_Kd(void)   { return Tel_Kd; }
uint8_t Read_Loop(void) { return loop;   }

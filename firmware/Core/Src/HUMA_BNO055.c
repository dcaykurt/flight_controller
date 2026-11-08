/*
 * HUMA_BNO055 — Bosch BNO055 IMU sürücüsü (I2C1)
 */
#include "HUMA_BNO055.h"
#include "i2c.h"

#define BNO055_I2C_ADDR         0x28   /* 7 bit adres; HAL için 1 bit sola kaydırılır */
#define BNO055_ADDR             (BNO055_I2C_ADDR << 1)

#define BNO055_CHIP_ID          0x00
#define BNO055_PAGE_ID          0x07
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_OPR_MODE         0x3D
#define BNO055_SYS_TRIGGER      0x3F

#define BNO055_EULER_OLCEK      16.0   /* 1 derece = 16 LSB */

typedef enum {
  BNO055_OPERATION_MODE_CONFIG = 0x00,
  /* Sensör modları */
  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYRONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_ACCGYRO,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG,           /* 0x07 */
  /* Füzyon modları */
  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF           /* 0x0C */
} bno055_opmode_t;

static uint8_t Reg_Buf[6];
static float   Roll_reel, Pitch_reel, Yaw_reel;

/* ------------------------------------------------------------------------- */
static void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
  HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDR, &reg, 1, 100);
  HAL_I2C_Master_Receive(&hi2c1, BNO055_ADDR, data, len, 100);
}

static void bno055_writeData(uint8_t reg, uint8_t data)
{
  uint8_t txdata[2] = {reg, data};
  HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDR, txdata, sizeof(txdata), 10);
}

static void bno055_setPage(uint8_t page)
{
  bno055_writeData(BNO055_PAGE_ID, page);
}

static void bno055_setOperationMode(bno055_opmode_t mode)
{
  bno055_writeData(BNO055_OPR_MODE, mode);
  /* Mod geçiş süreleri (datasheet): CONFIG'e 19 ms, diğerlerine 7 ms */
  HAL_Delay(mode == BNO055_OPERATION_MODE_CONFIG ? 19 : 7);
}

static void bno055_reset(void)
{
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  HAL_Delay(700);
}

static void bno055_setup(void)
{
  uint8_t id = 0;

  bno055_reset();
  bno055_readData(BNO055_CHIP_ID, &id, 1);   /* Beklenen: 0xA0 */
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
  HAL_Delay(10);
}

static uint8_t oku_bno1(uint8_t reg)
{
  uint8_t deger = 0;
  HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDR, &reg, 1, 10);
  HAL_I2C_Master_Receive(&hi2c1, BNO055_ADDR, &deger, 1, 10);
  return deger;
}

static float euler_oku(uint8_t lsb_reg, uint8_t msb_reg)
{
  uint8_t lsb = oku_bno1(lsb_reg);
  uint8_t msb = oku_bno1(msb_reg);
  int16_t ham = (int16_t)((msb << 8) | lsb);
  return ham / BNO055_EULER_OLCEK;
}

/* ------------------------------------------------------------------------- */
void HUMA_BNO055_Baslat(void)
{
  bno055_setup();
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

float HUMA_BNO055_EULER_ROLL(void)  { Roll_reel  = euler_oku(BNO055_EUL_ROLL_LSB,    BNO055_EUL_ROLL_MSB);    return Roll_reel;  }
float HUMA_BNO055_EULER_PITCH(void) { Pitch_reel = euler_oku(BNO055_EUL_PITCH_LSB,   BNO055_EUL_PITCH_MSB);   return Pitch_reel; }
float HUMA_BNO055_EULER_YAW(void)   { Yaw_reel   = euler_oku(BNO055_EUL_HEADING_LSB, BNO055_EUL_HEADING_MSB); return Yaw_reel;   }

/* Heading, roll, pitch (6 bayt) DMA ile okunur. DMA asenkron olduğundan
 * çözümlenen değerler bir önceki okuma döngüsüne aittir (10 ms gecikme). */
void HUMA_BNO055_ALL_READ(void)
{
  HAL_I2C_Mem_Read_DMA(&hi2c1, BNO055_ADDR, BNO055_EUL_HEADING_LSB, 1, Reg_Buf, 6);

  Yaw_reel   = (int16_t)((Reg_Buf[1] << 8) | Reg_Buf[0]) / BNO055_EULER_OLCEK;
  Roll_reel  = (int16_t)((Reg_Buf[3] << 8) | Reg_Buf[2]) / BNO055_EULER_OLCEK;
  Pitch_reel = (int16_t)((Reg_Buf[5] << 8) | Reg_Buf[4]) / BNO055_EULER_OLCEK;
}

float HUMA_Roll_Get(void)  { return Roll_reel;  }
float HUMA_Pitch_Get(void) { return Pitch_reel; }
float HUMA_Yaw_Get(void)   { return Yaw_reel;   }

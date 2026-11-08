/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * LOTUS — STM32F407 tabanlı quadcopter uçuş kontrol yazılımı
  * TEKNOFEST 2022 Döner Kanat (HUMA 2021 çalışmasının devamı)
  *
  * Görev dağılımı (tüm zamanlayıcılar 1 MHz tabanlı):
  *   TIM4  100 Hz : SBUS kumanda çerçevesini oku, arm/disarm kontrolü
  *   TIM5  100 Hz : BNO055 IMU'dan Euler açılarını oku
  *   TIM7  100 Hz : Kumanda eşleme + PID + motor karıştırma + ESC çıkışı
  *   TIM8    1 Hz : Arm/disarm için anahtar tutma süresi sayacı
  *   TIM3   50 Hz : 4 kanal ESC PWM (1000–2000 µs)
  ******************************************************************************
  Olmasa da canin sagolsun,
	Ucarsan da hakkindir...

	Hakkinla uc,
	Serefinle Kazan!
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "HUMA_BNO055.h"
#include "Huma_Telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Roll ve pitch açı PID'leri için ortak durum yapısı */
typedef struct {
  float error;
  float errorsum;
  float d_error;
  float lasterror;
  float p_out;
  float i_out;
  float d_out;
  float output;
} aci_pid_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Kontrol döngüsü periyodu (TIM7, 100 Hz) [s] */
#define KONTROL_DT              0.01

/* 1: PID katsayıları yer istasyonundan UART2 ile alınır (canlı ayar modu)
 * 0: Aşağıdaki sabit (uçuşta kullanılan) katsayılar kullanılır          */
#define PID_UART_AYAR           0

/* Uçuşta kullanılan PID katsayıları */
#define ROLL_PITCH_KP           1.65f
#define ROLL_PITCH_KI           0.35f
#define ROLL_PITCH_KD           0.5f
#define YAW_KP                  1.8f
#define YAW_KI                  0.2f
#define YAW_KD                  0.0f

/* Kumanda eşleme sınırları */
#define SBUS_MIN                306     /* SBUS ham kanal alt değeri */
#define SBUS_MAX                1693    /* SBUS ham kanal üst değeri */
#define ROLL_PITCH_MAKS_ACI     20      /* Hedef açı sınırı [derece]  */
#define YAW_MAKS_HIZ            180     /* Hedef yaw hızı [derece/s]  */
#define YAW_OLU_BOLGE           30      /* Yaw çubuğu ölü bölgesi     */

/* ESC / motor sınırları [µs] */
#define MOTOR_MIN               1200
#define MOTOR_MAX               1900
#define PID_AKTIF_GAZ           1250    /* Bu gazın altında PID sıfırlanır */

/* Arm / disarm */
#define ARM_SURESI              3       /* Anahtar tutma süresi [s] (+2 s güvenlik beklemesi) */
#define DISARM_SURESI           2       /* [s] */
#define ARM_ANAHTAR_ESIK        1600    /* Kanal 8 > eşik: arm isteği    */
#define DISARM_ANAHTAR_ESIK     400     /* Kanal 8 < eşik: disarm isteği */
#define ARM_MAKS_GAZ            1050    /* Arm için gaz çubuğu altta olmalı */

#define SBUS_CERCEVE_BOYU       22
#define SBUS_BASLIK             0x0F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* --- Arm / disarm --- */
int  KumandaSayac;                       /* TIM8 ile saniye sayar */
bool arm = false, arm_fonksiyon_kontrol = false;

/* --- SBUS kumanda --- */
char rec_sbus_data[SBUS_CERCEVE_BOYU];
int  sbus_channel[12];
int  roll_map, pitch_map, throttle_map, yaw_map;

/* --- PID --- */
aci_pid_t roll_pid, pitch_pid;
float Yaw_p_out, Yaw_i_out, Yaw_d_out, yaw_last_error, yawError, Yaw_output;
float saniyedeki_yaw_degisim_acisi;
float onceki_yaw;
int   roll_pid_gelen, pitch_pid_gelen, yaw_pid_gelen;

/* --- Motorlar --- */
int motor1, motor2, motor3, motor4;

/* --- IMU --- */
float IMU_Roll, IMU_Pitch, IMU_Yaw, roll_offset, pitch_offset;

/* --- UART2 yer istasyonu (PID canlı ayar) --- */
float Gelen_Kp, Gelen_Ki, Gelen_Kd;
char  rx_buffer[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int   HUMA_MAP(int gelen_veri, int giris_ilk, int giris_son, int cikis_ilk, int cikis_son);
static void  sbus_coz(void);
static void  huma_arm_kontrol(int arm_switchi);
static void  pwm_durdur(void);
static void  motor_sur(int m1, int m2, int m3, int m4, bool arm_check_val);
static int   motor_sinirla(int guc, int throttle);
static int   yaw_olu_bolge(int yaw_kumanda_degeri);
static float fix360derece(float deger);
static int   aci_pid_hesapla(aci_pid_t *pid, float kp, float ki, float kd,
                             float olculen_aci, int hedef_aci, int throttle);
static int   Yaw_pid_hesapla(float Yaw_kp, float Yaw_ki, float Yaw_kd,
                             float yaw_simdiki_aci, float yaw_onceki_aci,
                             float HedefYaw, float delta_t, int throttle);
static void  kontrol_dongusu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Doğrusal eşleme (Arduino map() eşdeğeri) */
static int HUMA_MAP(int gelen_veri, int giris_ilk, int giris_son, int cikis_ilk, int cikis_son)
{
  return ((gelen_veri - giris_ilk) * (cikis_son - cikis_ilk) / (giris_son - giris_ilk)) + cikis_ilk;
}

/* ---------------------------------------------------------------------------
 * SBUS: 25 baytlık çerçevenin ilk 22 baytı DMA ile alınır,
 * 11 bitlik kanallar bit kaydırma ile çözülür.
 * ------------------------------------------------------------------------- */
static void sbus_coz(void)
{
  const char *d = rec_sbus_data;
  sbus_channel[0]  = ((d[1]      | d[2]  << 8)                ) & 0x07FF;
  sbus_channel[1]  = ((d[2]  >> 3 | d[3]  << 5)                ) & 0x07FF;
  sbus_channel[2]  = ((d[3]  >> 6 | d[4]  << 2 | d[5]  << 10)  ) & 0x07FF;
  sbus_channel[3]  = ((d[5]  >> 1 | d[6]  << 7)                ) & 0x07FF;
  sbus_channel[4]  = ((d[6]  >> 4 | d[7]  << 4)                ) & 0x07FF;
  sbus_channel[5]  = ((d[7]  >> 7 | d[8]  << 1 | d[9]  << 9)   ) & 0x07FF;
  sbus_channel[6]  = ((d[9]  >> 2 | d[10] << 6)                ) & 0x07FF;
  sbus_channel[7]  = ((d[10] >> 5 | d[11] << 3)                ) & 0x07FF;
  sbus_channel[8]  = ((d[12]      | d[13] << 8)                ) & 0x07FF;
  sbus_channel[9]  = ((d[13] >> 3 | d[14] << 5)                ) & 0x07FF;
  sbus_channel[10] = ((d[14] >> 6 | d[15] << 2 | d[16] << 10)  ) & 0x07FF;
  sbus_channel[11] = ((d[16] >> 1 | d[17] << 7)                ) & 0x07FF;
}

static void pwm_durdur(void)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

/* ---------------------------------------------------------------------------
 * Arm / disarm
 *  - Arm   : kanal 8 yukarıda VE gaz altta, ARM_SURESI boyunca tutulmalı.
 *            Arm anında IMU açıları sıfır noktası (offset) olarak kaydedilir.
 *  - Disarm: kanal 8 aşağıda, DISARM_SURESI boyunca tutulmalı.
 *  Anahtar süre dolmadan bırakılırsa sayaç sıfırlanır.
 * ------------------------------------------------------------------------- */
static void huma_arm_kontrol(int arm_switchi)
{
  if (arm_fonksiyon_kontrol == false) {
    /* IMU kart üzerinde 90° döndürülmüş monte edildiği için eksenler yer değiştirir */
    roll_offset  = HUMA_Pitch_Get();
    pitch_offset = HUMA_Roll_Get();

    if (arm_switchi > ARM_ANAHTAR_ESIK && throttle_map < ARM_MAKS_GAZ) {
      HAL_TIM_Base_Start_IT(&htim8);
      if (KumandaSayac > ARM_SURESI) {
        HAL_GPIO_WritePin(GPIOD, led_arm_Pin, GPIO_PIN_SET);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
        arm = true;
        arm_fonksiyon_kontrol = true;
        HAL_TIM_Base_Stop_IT(&htim8);
        KumandaSayac = 0;
      }
    } else {
      /* Arm tamamlanmadan anahtar bırakıldı */
      pwm_durdur();
      HAL_TIM_Base_Stop(&htim8);
      KumandaSayac = 0;
    }
  }

  if (arm_fonksiyon_kontrol == true) {
    if (arm_switchi < DISARM_ANAHTAR_ESIK) {
      HAL_TIM_Base_Start_IT(&htim8);
      if (KumandaSayac > DISARM_SURESI) {
        pwm_durdur();
        arm = false;
        arm_fonksiyon_kontrol = false;
        HAL_GPIO_WritePin(GPIOD, led_arm_Pin, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop_IT(&htim8);
        KumandaSayac = 0;
      }
    } else {
      HAL_TIM_Base_Stop(&htim8);
      KumandaSayac = 0;
    }
  }
}

/* ESC çıkışları. PWM başlatma işlemi huma_arm_kontrol() içinde yapılır. */
static void motor_sur(int m1, int m2, int m3, int m4, bool arm_check_val)
{
  if (arm_check_val == false) {
    pwm_durdur();
  } else {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, m3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, m4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, m1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, m2);
  }
}

/* Motor gücünü [MOTOR_MIN, MOTOR_MAX] aralığına sınırlar.
 * Gaz MOTOR_MIN'in altındaysa PID katkısı uygulanmaz, gaz doğrudan geçer. */
static int motor_sinirla(int guc, int throttle)
{
  if (guc > MOTOR_MAX) guc = MOTOR_MAX;
  if (guc < MOTOR_MIN) guc = MOTOR_MIN;
  if (throttle < MOTOR_MIN) guc = throttle;
  return guc;
}

/* Yaw çubuğu merkeze (1000) YAW_OLU_BOLGE kadar yakınsa merkez kabul edilir */
static int yaw_olu_bolge(int yaw_kumanda_degeri)
{
  if (abs(yaw_kumanda_degeri - 1000) < YAW_OLU_BOLGE) {
    return 1000;
  }
  return yaw_kumanda_degeri;
}

/* Açı farkını [-180, 180] aralığına sarar */
static float fix360derece(float deger)
{
  if (deger > 180)  return deger - 360;
  if (deger < -180) return deger + 360;
  return deger;
}

/* ---------------------------------------------------------------------------
 * Roll / pitch açı PID'i
 *  - İntegral yalnızca |hata| < 5° iken biriktirilir (anti-windup),
 *    hata büyüyünce sıfırlanır.
 *  - Gaz PID_AKTIF_GAZ altındayken tüm terimler sıfırlanır (yerde).
 *  - Çıkış ±kp*2*ROLL_PITCH_MAKS_ACI ile sınırlanır.
 * ------------------------------------------------------------------------- */
static int aci_pid_hesapla(aci_pid_t *pid, float kp, float ki, float kd,
                           float olculen_aci, int hedef_aci, int throttle)
{
  pid->error = hedef_aci - olculen_aci;
  pid->p_out = kp * pid->error;

  pid->d_error   = (pid->error - pid->lasterror) / KONTROL_DT;
  pid->lasterror = pid->error;
  pid->d_out     = kd * pid->d_error;

  if (pid->error > 5 || pid->error < -5) {
    pid->errorsum = 0;
    pid->i_out    = 0;
  } else if (pid->error < 5 && pid->error > -5) {
    pid->errorsum += pid->error * KONTROL_DT;
    pid->i_out     = ki * pid->errorsum;
  }

  if (throttle < PID_AKTIF_GAZ) {
    pid->errorsum = 0;
    pid->d_error  = 0;
    pid->i_out    = 0;
    pid->p_out    = 0;
    pid->d_out    = 0;
  }

  pid->output = pid->p_out + pid->i_out + pid->d_out;

  const float sinir = kp * ROLL_PITCH_MAKS_ACI * 2;
  if (pid->output >  sinir) pid->output =  sinir;
  if (pid->output < -sinir) pid->output = -sinir;

  return pid->output;
}

/* ---------------------------------------------------------------------------
 * Yaw hız PID'i: hedef, yaw açısının saniyedeki değişimi (derece/s)
 * ------------------------------------------------------------------------- */
static int Yaw_pid_hesapla(float Yaw_kp, float Yaw_ki, float Yaw_kd,
                           float yaw_simdiki_aci, float yaw_onceki_aci,
                           float HedefYaw, float delta_t, int throttle)
{
  float aci_degisimi = fix360derece(yaw_simdiki_aci - yaw_onceki_aci);
  saniyedeki_yaw_degisim_acisi = aci_degisimi / delta_t;
  yawError = HedefYaw - saniyedeki_yaw_degisim_acisi;

  if (yawError < -180) yawError = -180;
  if (yawError >  180) yawError =  180;

  Yaw_p_out  = yawError * Yaw_kp;
  Yaw_i_out += yawError * delta_t;
  Yaw_i_out  = Yaw_i_out * Yaw_ki;
  Yaw_d_out  = Yaw_kd * ((yawError - yaw_last_error) / delta_t);
  yaw_last_error = yawError;

  if (throttle < PID_AKTIF_GAZ) {
    yawError  = 0;
    Yaw_i_out = 0;
    Yaw_p_out = 0;
    Yaw_d_out = 0;
    saniyedeki_yaw_degisim_acisi = 0;
  }

  Yaw_output = Yaw_p_out + Yaw_i_out + Yaw_d_out;

  const float sinir = Yaw_kp * 45 * 2;
  if (Yaw_output >  sinir) Yaw_output =  sinir;
  if (Yaw_output < -sinir) Yaw_output = -sinir;

  return Yaw_output;
}

/* ---------------------------------------------------------------------------
 * 100 Hz ana kontrol döngüsü:
 * kumanda -> hedef açı/hız -> PID -> X-quad motor karıştırma -> ESC
 * ------------------------------------------------------------------------- */
static void kontrol_dongusu(void)
{
  roll_map     = HUMA_MAP(sbus_channel[0], SBUS_MAX, SBUS_MIN, ROLL_PITCH_MAKS_ACI, -ROLL_PITCH_MAKS_ACI);
  pitch_map    = HUMA_MAP(sbus_channel[1], SBUS_MIN, SBUS_MAX, ROLL_PITCH_MAKS_ACI, -ROLL_PITCH_MAKS_ACI);
  throttle_map = HUMA_MAP(sbus_channel[2], SBUS_MAX, SBUS_MIN, 1000, 2000);
  yaw_map      = HUMA_MAP(yaw_olu_bolge(sbus_channel[3]), SBUS_MIN, SBUS_MAX, -YAW_MAKS_HIZ, YAW_MAKS_HIZ);

#if PID_UART_AYAR
  const float rp_kp = Gelen_Kp, rp_ki = Gelen_Ki, rp_kd = Gelen_Kd;
#else
  const float rp_kp = ROLL_PITCH_KP, rp_ki = ROLL_PITCH_KI, rp_kd = ROLL_PITCH_KD;
#endif

  roll_pid_gelen  = aci_pid_hesapla(&roll_pid,  rp_kp, rp_ki, rp_kd, IMU_Roll,  roll_map,  throttle_map);
  pitch_pid_gelen = aci_pid_hesapla(&pitch_pid, rp_kp, rp_ki, rp_kd, IMU_Pitch, pitch_map, throttle_map);
  yaw_pid_gelen   = Yaw_pid_hesapla(YAW_KP, YAW_KI, YAW_KD, IMU_Yaw, onceki_yaw, yaw_map, KONTROL_DT, throttle_map);
  onceki_yaw = IMU_Yaw;

  /* X konfigürasyon motor karıştırma */
  const int r = roll_pid_gelen, p = pitch_pid_gelen, y = yaw_pid_gelen, t = throttle_map;
  motor1 = motor_sinirla(t + y + p + r, t);
  motor2 = motor_sinirla(t - y - p + r, t);
  motor3 = motor_sinirla(t + y - p - r, t);
  motor4 = motor_sinirla(t - y + p - r, t);

  motor_sur(motor1, motor2, motor3, motor4, arm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);   /* SBUS + arm kontrolü */
  HAL_TIM_Base_Start_IT(&htim7);   /* Kontrol döngüsü     */
  HAL_TIM_Base_Start_IT(&htim5);   /* IMU okuma           */
  HAL_Delay(1000);
  HUMA_BNO055_Baslat();
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer, sizeof(rx_buffer));
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Tüm görevler zamanlayıcı kesmelerinde çalışır */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Yer istasyonundan PID katsayısı paketi ("K..L..M..N") */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    veri_ayir(rx_buffer);
    Gelen_Kp = Read_Kp();
    Gelen_Ki = Read_Ki();
    Gelen_Kd = Read_Kd();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {          /* 100 Hz: kumanda */
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rec_sbus_data, SBUS_CERCEVE_BOYU);
    if (rec_sbus_data[0] == SBUS_BASLIK) {
      sbus_coz();
      huma_arm_kontrol(sbus_channel[7]);
    }
  }

  if (htim->Instance == TIM5) {          /* 100 Hz: IMU */
    HUMA_BNO055_ALL_READ();
    /* IMU 90° döndürülmüş monte edildiği için roll/pitch eksenleri yer değiştirir */
    IMU_Roll  = HUMA_Pitch_Get() - roll_offset;
    IMU_Pitch = HUMA_Roll_Get()  - pitch_offset;
    IMU_Yaw   = HUMA_Yaw_Get();
  }

  if (htim->Instance == TIM7) {          /* 100 Hz: kontrol */
    kontrol_dongusu();
  }

  if (htim->Instance == TIM8) {          /* 1 Hz: arm/disarm süre sayacı */
    KumandaSayac += 1;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

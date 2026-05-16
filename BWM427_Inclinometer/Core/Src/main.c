/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ili9341.h"     // Подключить драйвер дисплея
#include "fonts.h"       // Подключить шрифты дисплея

// Структура для модуля часов (RTC)
typedef struct {
	uint8_t seconds, minutes, hours, day, date, month, year;
} RTC_Time;

RTC_Time current_time; // Глобальная переменная для хранения времени

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- Данные датчика и математика ---
uint8_t rx_buffer[9];
float angle_x = 0.0f, angle_y = 0.0f;
float offset_x = 0.0f, offset_y = 0.0f;
uint8_t modbus_ready = 0;

// --- EMA фильтрация ---
float ema_alpha = 0.1f; // Коэффициент сглаживания (от 0.01 до 0.99)
float ema_x = 0.0f, ema_y = 0.0f;
uint8_t ema_initialized = 0;

// --- Файловая система (SD карта) ---
FATFS fs;
FIL fil;
uint8_t is_recording = 0;
uint16_t file_number = 0;
uint8_t sd_err_code = 0; // Переменная для кода ошибки
char current_filename[16];

// --- �?нтерфейс и энкодер ---
int16_t last_encoder_cnt = 0;
int8_t menu_index = 0;
uint8_t edit_mode = 0;
uint16_t log_freq_hz = 10;
uint32_t log_delay_ms = 100;
int8_t prev_menu_index = -1;
uint8_t prev_edit_mode = 255;

// --- Мониторинг питания ---
float battery_v = 0.0f;

// --- Настройка RS485 (MAX485) ---
#define RS485_DE_PIN  RS485_DE_Pin
#define RS485_DE_PORT RS485_DE_GPIO_Port
#define RS485_RE_PIN  RS485_RE_Pin
#define RS485_RE_PORT RS485_RE_GPIO_Port

// --- Системные таймеры и алгоритм стабильности ---
uint32_t record_start_ms = 0;
float stability_buffer[20] = { 0 };
uint8_t buffer_idx = 0;
uint8_t is_stable = 0;
uint32_t lcd_timer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
// Опросить датчик по протоколу Modbus RTU
void BWM427_ReadAngles(void);

// Применить экспоненциальное сглаживание для фильтрации данных
float apply_ema(float old_value, float new_value, float alpha);

// Рассчитать контрольную сумму CRC16 для пакета Modbus
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);

// Считать текущее время с модуля RTC (I2C)
void DS3231_GetTime(RTC_Time *time);
void DS3231_SetFullTime(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t mo,
		uint8_t y);

// Отрисовать прямоугольную рамку на дисплее
void My_DrawFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color);

uint8_t Dec_To_BCD(uint8_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	// Выждать время для стабилизации питания и инициализации USB
	HAL_Delay(1000);

	// === �?Н�?Ц�?АЛ�?ЗАЦ�?Я Д�?СПЛЕЯ ===
	ILI9341_Init();
	ILI9341_FillScreen(0x0000);

	// Запустить таймер энкодера (аппаратный подсчет импульсов)
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	// Запустить АЦП для мониторинга напряжения батареи
	HAL_ADC_Start(&hadc1);

	// Считать начальное значение энкодера
	last_encoder_cnt = __HAL_TIM_GET_COUNTER(&htim3);

	// �?нициализировать SD-карту
	HAL_Delay(500);
	FRESULT res = f_mount(&fs, "", 1);
	if (res == FR_OK) {
		FILINFO fno;
		for (uint16_t i = 1; i <= 999; i++) {
			sprintf(current_filename, "M_%03d.CSV", i);
			if (f_stat(current_filename, &fno) != FR_OK) {
				file_number = i;
				break;
			}
		}
	} else {
		file_number = 0;
		sd_err_code = res; // Сохраняем код ошибки!
	}
	// === �?НД�?КАЦ�?Я УСПЕШНОГО ЗАПУСКА ===
	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Вкл
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Выкл
		HAL_Delay(100);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Умная настройка времени (сработает только если часы сброшены)
	DS3231_GetTime(&current_time);
	if (current_time.year < 81) {
		// Если год меньше 26, прошиваем время (например: 11:11:00, 11 сентября 01 года)
		DS3231_SetFullTime(11, 49, 00, 16, 5, 26);
	}

	static uint8_t last_sw_state = 0; // Сохранить прошлое состояние тумблера

	while (1) {
		char str_lcd[64];
		DS3231_GetTime(&current_time); // Получаем время с часов

		// 1. ОБРАБОТКА ЭНКОДЕРА (Защита от скачков и деление на 4)
		int16_t current_cnt = __HAL_TIM_GET_COUNTER(&htim3);
		int16_t raw_diff = (int16_t) (current_cnt - last_encoder_cnt);

		if (raw_diff >= 4 || raw_diff <= -4) {
			int16_t diff = (raw_diff / 4); // Реверс и деление на 4
			last_encoder_cnt = current_cnt;

			if (edit_mode) {
				if (menu_index == 0) { // Настройка частоты
					log_freq_hz += diff;
					if (log_freq_hz < 1)
						log_freq_hz = 1;
					if (log_freq_hz > 50)
						log_freq_hz = 50;
					log_delay_ms = 1000 / log_freq_hz;
				} else if (menu_index == 1) { // Настройка Alpha фильтра
					ema_alpha += (float) diff * 0.01f;
					if (ema_alpha < 0.01f)
						ema_alpha = 0.01f;
					if (ema_alpha > 0.99f)
						ema_alpha = 0.99f;
				}
			} else {
				menu_index += diff;
				while (menu_index < 0)
					menu_index += 3;
				while (menu_index > 2)
					menu_index -= 3;
			}
		}

		// 2. МГНОВЕННАЯ ОТР�?СОВКА РАМК�? (Эстетичная сетка)
		if (menu_index != prev_menu_index || edit_mode != prev_edit_mode) {
			// Координаты для 3 пунктов: 0-Freq, 1-Alpha, 2-Zero
			const uint16_t frame_x[3] = { 15, 165, 75 };
			const uint16_t frame_y[3] = { 145, 145, 185 };
			const uint16_t frame_w[3] = { 115, 125, 170 };

			if (prev_menu_index != -1) {
				// Стираем старую рамку цветом фона (черным)
				My_DrawFrame(frame_x[prev_menu_index], frame_y[prev_menu_index],
						frame_w[prev_menu_index], 24, 0x0000);
			}

			// Рисуем новую рамку (Желтая - редактирование, Голубая - выбор)
			uint16_t frame_color = edit_mode ? 0xFFE0 : 0x07FF;
			My_DrawFrame(frame_x[menu_index], frame_y[menu_index],
					frame_w[menu_index], 24, frame_color);

			prev_menu_index = menu_index;
			prev_edit_mode = edit_mode;
		}

		// 3. УПРАВЛЕН�?Е ЗАП�?СЬЮ (Тумблер на PA3 - срабатывает при замыкании на GND)
		uint8_t current_sw = HAL_GPIO_ReadPin(SW_RECORD_GPIO_Port,
		SW_RECORD_Pin);

		if (current_sw == 0 && last_sw_state == 1) { // Замкнули на землю
			if (f_open(&fil, current_filename, FA_CREATE_ALWAYS | FA_WRITE)
					== FR_OK) {
				record_start_ms = HAL_GetTick();
				char header[] =
						"Time,RawX,RawY,OffsetX,OffsetY,CalcX,CalcY,BatV\n";
				UINT bw;
				f_write(&fil, header, strlen(header), &bw);
				is_recording = 1;
			}
		} else if (current_sw == 1 && last_sw_state == 0) { // Разомкнули
			if (is_recording) {
				f_close(&fil);
				is_recording = 0;
				file_number++;
				sprintf(current_filename, "M_%03d.CSV", file_number);
			}
		}
		last_sw_state = current_sw;

		// 4. ОПРОС ДАТЧ�?КА �? РАСЧЕТ СТАБ�?ЛЬНОСТ�?
		static uint32_t sensor_timer = 0;
		if (HAL_GetTick() - sensor_timer >= log_delay_ms) {
			BWM427_ReadAngles();
			float calc_x = angle_x - offset_x;
			float calc_y = angle_y - offset_y;

			stability_buffer[buffer_idx] = calc_x;
			buffer_idx = (buffer_idx + 1) % 20;

			float min_v = stability_buffer[0], max_v = stability_buffer[0];
			for (int i = 1; i < 20; i++) {
				if (stability_buffer[i] < min_v)
					min_v = stability_buffer[i];
				if (stability_buffer[i] > max_v)
					max_v = stability_buffer[i];
			}
			is_stable = (max_v - min_v < 0.05f) ? 1 : 0;

			if (is_recording) {
				char buf[128];
				sprintf(buf,
						"%02d.%02d.%02d %02d:%02d:%02d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n",
						current_time.date, current_time.month,
						current_time.year, current_time.hours,
						current_time.minutes, current_time.seconds, angle_x,
						angle_y, offset_x, offset_y, calc_x, calc_y, battery_v);
				UINT bw;
				f_write(&fil, buf, strlen(buf), &bw);
				//f_sync(&fil);
			}
			sensor_timer = HAL_GetTick();
		}

		// 5. МОН�?ТОР�?НГ БАТАРЕ�? (Новый коэффициент для 10к и 3.3к)
		static uint32_t bat_timer = 0;
		if (HAL_GetTick() - bat_timer >= 1000) {
			if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
				uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
				// Коэффициент 4.03f для твоего делителя
				battery_v = ((float) adc_val / 4095.0f) * 3.3f * 4.03f;
			}
			HAL_ADC_Start(&hadc1);
			bat_timer = HAL_GetTick();
		}

		// 6. ОБНОВЛЕН�?Е ЭКРАНА (Новый просторный дизайн)
		if (HAL_GetTick() - lcd_timer >= 50) {
			static float old_x = -999.0f, old_y = -999.0f;
			float cur_x = angle_x - offset_x;
			float cur_y = angle_y - offset_y;

			// Углы (Сдвинуты в левую часть, много свободного места)
			if (fabsf(cur_x - old_x) > 0.005f) {
				sprintf(str_lcd, "X: %+7.3f ", cur_x);
				ILI9341_WriteString(10, 40, str_lcd, Font_16x26, 0xFFFF,
						0x0000);
				old_x = cur_x;
			}
			if (fabsf(cur_y - old_y) > 0.005f) {
				sprintf(str_lcd, "Y: %+7.3f ", cur_y);
				ILI9341_WriteString(10, 80, str_lcd, Font_16x26, 0xFFFF,
						0x0000);
				old_y = cur_y;
			}

			// Часы и дата
			sprintf(str_lcd, "%02d:%02d:%02d", current_time.hours,
					current_time.minutes, current_time.seconds);
			ILI9341_WriteString(220, 5, str_lcd, Font_11x18, 0xFFFF, 0x0000);
			sprintf(str_lcd, "%02d.%02d.20%02d", current_time.date,
					current_time.month, current_time.year);
			ILI9341_WriteString(220, 25, str_lcd, Font_7x10, 0x07FF, 0x0000);

			// �?ндикатор записи (Под датой справа)
			if (is_recording) {
				if ((HAL_GetTick() / 500) % 2) {
					ILI9341_WriteString(240, 45, "REC", Font_11x18, 0xF800,
							0x0000);
				} else {
					ILI9341_WriteString(240, 45, "   ", Font_11x18, 0xFFFF,
							0x0000);
				}
			} else {
				ILI9341_WriteString(240, 45, "STOP", Font_11x18, 0x7BEF, 0x0000);
			}

			// Меню
			sprintf(str_lcd, " Freq:%02d ", log_freq_hz);
			ILI9341_WriteString(20, 148, str_lcd, Font_11x18, 0xFFFF, 0x0000);

			sprintf(str_lcd, " Alp:%.2f ", ema_alpha);
			ILI9341_WriteString(170, 148, str_lcd, Font_11x18, 0xFFFF, 0x0000);

			ILI9341_WriteString(80, 188, " [ SET ZERO ] ", Font_11x18, 0xFFFF,
					0x0000);

			// Статус бар (Самая нижняя строка)
			sprintf(str_lcd, "BAT:%.1fV", battery_v);
			ILI9341_WriteString(5, 220, str_lcd, Font_7x10, 0xFFFF, 0x0000);

			// Проверка наличия SD карты
			if (file_number > 0) {
				sprintf(str_lcd, "FILE:%s", current_filename);
				ILI9341_WriteString(90, 220, str_lcd, Font_7x10, 0xFFE0,
						0x0000);
			} else {
				// Выводим код ошибки на экран
				sprintf(str_lcd, "SD ERR:%d", sd_err_code);
				ILI9341_WriteString(90, 220, str_lcd, Font_7x10, 0xF800,
						0x0000);
			}

			if (is_stable) {
				ILI9341_WriteString(240, 220, " READY ", Font_7x10, 0x07E0,
						0x0000);
			} else {
				ILI9341_WriteString(240, 220, " WAIT..", Font_7x10, 0xF800,
						0x0000);
			}

			lcd_timer = HAL_GetTick();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_RECORD_Pin */
  GPIO_InitStruct.Pin = SW_RECORD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_RECORD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_DC_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_DC_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_RE_Pin */
  GPIO_InitStruct.Pin = RS485_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_RE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
// Настройка встроенного светодиода (PC13)
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

// Дополнительная инициализация PB15 для RS485
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 1.Конвертер из формата часов (BCD) в обычные числа
uint8_t BCD_To_Dec(uint8_t val) {
	return (uint8_t) ((val / 16 * 10) + (val % 16));
}
// Конвертер из обычных чисел в формат BCD
uint8_t Dec_To_BCD(uint8_t val) {
	return (uint8_t) ((val / 10 * 16) + (val % 10));
}

// 2. Обновленное чтение ВСЕХ параметров времени и даты
void DS3231_GetTime(RTC_Time *time) {
	uint8_t g[7];
// 0xD0 - адрес, 0x00 - стартовый регистр, 7 байт данных
	if (HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x00, 1, g, 7, 1000) == HAL_OK) {
		time->seconds = BCD_To_Dec(g[0]);
		time->minutes = BCD_To_Dec(g[1]);
		time->hours = BCD_To_Dec(g[2] & 0x3F);
		time->date = BCD_To_Dec(g[4]);
		time->month = BCD_To_Dec(g[5] & 0x7F);
		time->year = BCD_To_Dec(g[6]);
	}

}

// 3. Функция для разовой установки времени
void DS3231_SetFullTime(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t mo,
		uint8_t y) {
	uint8_t t[7];
	t[0] = Dec_To_BCD(s);
	t[1] = Dec_To_BCD(m);
	t[2] = Dec_To_BCD(h);
	t[3] = 1; // День недели (просто 1)
	t[4] = Dec_To_BCD(d);
	t[5] = Dec_To_BCD(mo);
	t[6] = Dec_To_BCD(y);
	HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x00, 1, t, 7, 1000);
}

// Рассчитать контрольную сумму CRC16 для протокола Modbus RTU
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for (uint16_t pos = 0; pos < len; pos++) {
		crc ^= (uint16_t) buf[pos];
		for (int i = 8; i != 0; i--) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc; // Возвращает Low byte в младшем байте, High byte в старшем
}

// Применить экспоненциальное сглаживание (фильтр EMA)
float apply_ema(float old_value, float new_value, float alpha) {
	return (alpha * new_value) + ((1.0f - alpha) * old_value);
}

// Опросить датчик BWM427 по интерфейсу RS485 (Modbus RTU)
void BWM427_ReadAngles(void) {
// Сформировать запрос: ID=01, Func=03, Reg=0001, Count=0002
	uint8_t cmd[8] = { 0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00 };
	uint16_t crc = Modbus_CRC16(cmd, 6);
	cmd[6] = (uint8_t) (crc & 0xFF);
	cmd[7] = (uint8_t) ((crc >> 8) & 0xFF);

// Переключить MAX485 в режим передачи (DE=1, RE=1)
	HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, cmd, 8, 10);

// Дождаться завершения физической передачи последнего бита
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
		;

// Переключить MAX485 в режим приема (DE=0, RE=0)
	HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);

// Принять ответ (9 байт)
	if (HAL_UART_Receive(&huart1, rx_buffer, 9, 50) == HAL_OK) {
		// Проверить контрольную сумму ответа
		uint16_t received_crc = (rx_buffer[8] << 8) | rx_buffer[7];
		if (Modbus_CRC16(rx_buffer, 7) == received_crc) {

			// �?звлечь сырые данные и перевести в градусы
			int16_t rx_x = (int16_t) ((rx_buffer[3] << 8) | rx_buffer[4]);
			int16_t rx_y = (int16_t) ((rx_buffer[5] << 8) | rx_buffer[6]);

			float raw_angle_x = (float) (rx_x - 10000) / 100.0f;
			float raw_angle_y = (float) (rx_y - 10000) / 100.0f;

			// Выполнить фильтрацию EMA
			if (!ema_initialized) {
				ema_x = raw_angle_x;
				ema_y = raw_angle_y;
				ema_initialized = 1;
			} else {
				ema_x = apply_ema(ema_x, raw_angle_x, ema_alpha); // <--- ЗДЕСЬ
				ema_y = apply_ema(ema_y, raw_angle_y, ema_alpha); // <--- �? ЗДЕСЬ
			}
			angle_x = ema_x;
			angle_y = ema_y;
			modbus_ready = 1;
		}
	}
}

// Обработать прерывание от кнопки энкодера (PA0)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ENCODER_SW_Pin) {
		static uint32_t last_press_time = 0;
		if (HAL_GetTick() - last_press_time > 200) {

			if (menu_index == 2) {
				offset_x = angle_x;
				offset_y = angle_y;
			} else {
				edit_mode = !edit_mode;
			}

			last_press_time = HAL_GetTick();
		}
	}
}

// Функция для рисования рамки вокруг выбранного пункта
void My_DrawFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color) {
	uint8_t thickness = 2; // Толщина рамки в пикселях (можешь поставить 3)

	for (uint8_t t = 0; t < thickness; t++) {
		// Рисуем горизонтальные линии (верх и низ)
		for (uint16_t i = x; i < x + w; i++) {
			ILI9341_DrawPixel(i, y + t, color);
			ILI9341_DrawPixel(i, y + h - t, color);
		}
		// Рисуем вертикальные линии (лево и право)
		for (uint16_t i = y; i < y + h; i++) {
			ILI9341_DrawPixel(x + t, i, color);
			ILI9341_DrawPixel(x + w - t, i, color);
		}
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
	__disable_irq();
	while (1) {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

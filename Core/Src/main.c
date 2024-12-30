/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "myDS1302Lib.h"
#include "RGB.h"
#include "string.h"
#include "fatfs_sd.h"
#include "stdio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_SIZE		10
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// UART Interrupt İslemi Global Degiskenleri
uint8_t rxBuffer;
uint8_t receivedData[BUFFER_SIZE]; // Gelen veriyi saklamak için
uint8_t dataIndex = 0;


// MicroSD Global Kart Degiskenleri
SPI_HandleTypeDef hspi3;
char TxBuffer[250];

// RTC bilgisini SD karta yazmak icin global degiskenler
FATFS FatFs;
FIL Fil;
FRESULT FR_Status;

// RTC Global Degiskenleri
uint8_t time_buffer[8];

// buzzer state machine Global degiskenleri
uint8_t buzzerState = 0;
uint32_t buzzerFirstTime = 0;
uint32_t buzzerSecondTime = 0;

// rbg led state machine global degiskenleri
uint8_t rgb_state = 0;
uint32_t rgb_first_time = 0;

// RGB led global degiskenleri
RGB_LED my_rgb_led;
TIM_HandleTypeDef htim3;

// ADC okuma degiskeni
volatile uint32_t adcBuf;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
static void SD_Card_Test(void);
static void writeTimeToSD(void);
static void SD_Card_TimeLogger(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART_Print(char* str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen(str), 100);
}


// Bu fonksiyon islemcinin UID'sini gormek icin
static void print_UID(void)
{
	char uidBuf[50];
	uint32_t uid[3];

	uid[0] = *(uint32_t*)0x1FFF7A10;
	uid[1] = *(uint32_t*)0x1FFF7A14;
	uid[2] = *(uint32_t*)0x1FFF7A18;

	sprintf(uidBuf, "Unique ID: 0x%08lX-%08lX-%08lX",
	            uid[0],
	            uid[1],
	            uid[2]);

	UART_Print(uidBuf);
}

void send_adc_value()
{
	char buf[50];
	sprintf(buf,"ADC degeri = %lu\r\n",adcBuf);
	UART_Print(buf);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  // Islemcinin UID'sini oku
  // print_UID();

  // RTC konfigürasyon ve set etme
  DS1302_Init();
  DS1302_WriteByte(DS1302_CONTROL, 0x00);	// Yazma kontrolunu kaldir
  uint8_t set_time[8] = {0, 24, 12, 16, 20, 52, 45, 2};
  DS1302_WriteTime(set_time);

  // 1 byte UART interrupt islemini baslat
  HAL_UART_Receive_IT(&huart2, &rxBuffer, 1);

  // RGB ledler icin gerekli PWM'leri baslat
  RGB_LED_Init(&my_rgb_led, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);	// timer3 ch1,ch2,ch3

  // 1 saniye buzzer'ı çal
  switch (buzzerState)
  {
  	  case 0:
  	  {
  		  buzzerState = 1;
  	  }break;
  	  case 1:
  	  {
  		  buzzerFirstTime = HAL_GetTick();
  		  buzzerState = 2;
  		  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// Buzzer cal
  	  }break;
  	  case 2:
  	  {
  		  buzzerSecondTime = HAL_GetTick();
  		  if (buzzerSecondTime -  buzzerFirstTime > 1000)
  		  {
  			// HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);	// Buzzer durdur
  			buzzerState = 0;
  		  }
  	  }break;
  }

  // Zamani SD Karta loglama fonksiyonu,calisması yaklasik 10 saniye suruyor
  // SD_Card_TimeLogger();
  // SD Kart Test Fonksiyonu, ne kadar free alan oldugunu hesapliyor
  // SD_Card_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // ADC IT ve timer calistir
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_IT(&hadc1);

  while (1)
  {
	  send_adc_value();
	  HAL_Delay(1000);
//	  switch(rgb_state)
//	  {
//	      case 0:  // Başlangıç durumu
//	      {
//	          rgb_state = 1;
//	          rgb_first_time = HAL_GetTick();
//	      } break;
//
//	      case 1:  // Kırmızı
//	      {
//	          if (HAL_GetTick() - rgb_first_time > 1000)
//	          {
//	              RGB_LED_Set_Color(&my_rgb_led, 255, 0, 128);
//	              rgb_first_time = HAL_GetTick();
//	              rgb_state = 2;
//	          }
//	      } break;
//
//	      case 2:  // Yeşil
//	      {
//	          if (HAL_GetTick() - rgb_first_time > 1000)
//	          {
//	              RGB_LED_Set_Color(&my_rgb_led, 255, 192, 103);
//	              rgb_first_time = HAL_GetTick();
//	              rgb_state = 3;
//	          }
//	      } break;
//
//	      case 3:  // Mavi
//	      {
//	          if (HAL_GetTick() - rgb_first_time > 1000)
//	          {
//	              RGB_LED_Set_Color(&my_rgb_led, 50, 60, 70);
//	              rgb_first_time = HAL_GetTick();
//	              rgb_state = 0;
//	          }
//	      } break;
//	  }
//	  RGB_LED_Set_Color(&my_rgb_led, 0, 255, 25);
//	  HAL_Delay(100);
//	  RGB_LED_Set_Color(&my_rgb_led, 255, 0, 255);
//	  HAL_Delay(100);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Gelen veriyi buffer'a ekle
        receivedData[dataIndex++] = rxBuffer;

        // Eger buffer dolduysa, basa sar
        if (dataIndex >= BUFFER_SIZE)
        {
        	dataIndex = 0; // Index degerini sıfırla
        }

        // Bir sonraki veriyi bekle
        HAL_UART_Receive_IT(&huart2, &rxBuffer, 1);
    }
}

static void SD_Card_Test(void)
{
  FATFS FatFs;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;

  // SD Kart Yerlestir
  FR_Status = f_mount(&FatFs, "", 1);
  if (FR_Status != FR_OK)
  {
    sprintf(TxBuffer, "Hata! SD Kart yerleştirilemedi. Hata kodu: (%i)\r\n", FR_Status);
    UART_Print(TxBuffer);
  }
  else
  {
	  sprintf(TxBuffer, "SD Kart Basariyla Yerlestirildi! \r\n\n");
	  UART_Print(TxBuffer);
  }

  //SD Kart Hafıza Durumu kontrolü
  f_getfree("", &FreeClusters, &FS_Ptr);

  TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
  FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
  sprintf(TxBuffer, "Total SD Kart Boyutu: %lu Byte\r\n", TotalSize);
  UART_Print(TxBuffer);
  sprintf(TxBuffer, "Boş SD Kart Boyutu: %lu Byte\r\n\n", FreeSpace);
  UART_Print(TxBuffer);

  // Test tamamlandi
  FR_Status = f_mount(NULL, "", 0);

  if (FR_Status != FR_OK)
  {
      sprintf(TxBuffer, "Hata! SD Kart cikartilamadi, Hata kodu: (%i)\r\n", FR_Status);
      UART_Print(TxBuffer);
  }
  else
  {
      sprintf(TxBuffer, "SD Kart basariyla kaldirildi! \r\n");
      UART_Print(TxBuffer);
  }
}

static void writeTimeToSD(void)
{
	// Zaman bilgisini tutmak icin
    uint8_t time_buffer_rtc[8];
    // SD karta yazmak icin
    char time_buffer[64];

    DS1302_ReadTime(time_buffer_rtc);

    // Zaman bilgisini yazabilecek formata cevir
    sprintf(time_buffer, "%02d-%02d-%02d %02d:%02d:%02d\r\n",
            2000 + time_buffer_rtc[1], time_buffer_rtc[2], time_buffer_rtc[3],
            time_buffer_rtc[4], time_buffer_rtc[5], time_buffer_rtc[6]);

    FR_Status = f_open(&Fil, "TimeLog.txt", FA_OPEN_APPEND | FA_WRITE);

    if (FR_Status == FR_OK)
    {
        // Zaman bilgisini dosyaya yaz
        f_puts(time_buffer, &Fil);
        // Yazilan zaman bilgisini bluetooth uzerinden gonder
        UART_Print(time_buffer);
        f_close(&Fil);
    }
    else
    {
        // Hata mesaji
    	UART_Print("Hata! Zaman bilgisi yazilamadi...");
    }
}

static void SD_Card_TimeLogger(void)
{

	int counter = 0;
	const int maxWrites = 10;

	// SD kartı yerlestir
    FR_Status = f_mount(&FatFs, "", 1);
    if (FR_Status != FR_OK)
    {
        UART_Print("Hata! SD Kart Yerlestirilemedi...");
        return;
    }

    // Durmadan yazma yapma, maxWrite kadar yaz ve SD karti kaldir
    while (counter < maxWrites)
    {
    	writeTimeToSD(); // Zamanı SD karta yaz
        HAL_Delay(1000); // 1 saniye bekle
        counter++;       // Sayacı artir
    }

    // SD kartı kaldir
    f_mount(NULL, "", 0);
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

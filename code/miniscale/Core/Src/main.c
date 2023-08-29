/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "i2c_ex.h"
#include "sk6812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
union
{
  float f;
  uint8_t c[4];
}gap_value;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 1
#define I2C_ADDRESS 0x26
#define FLASH_DATA_SIZE 8
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000) 

#define SAMPLE_TIME 11

#define HX711_getDataValue() (!!(GPIOA->IDR & GPIO_PIN_5))
#define HX711_setClkValue_High GPIOA->BSRR = GPIO_PIN_6
#define HX711_setClkValue_Low GPIOA->BRR = GPIO_PIN_6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t fm_version = FIRMWARE_VERSION;
uint8_t i2c_address[1] = {0};
uint8_t flash_data[FLASH_DATA_SIZE] = {0};

volatile int32_t offset_adc = 0;
volatile int32_t raw_adc = 0;
volatile int32_t raw_adc_set[SAMPLE_TIME] = {0};
volatile float weight_value = 0;

volatile uint8_t led_r = 0;
volatile uint8_t led_g = 0;
volatile uint8_t led_b = 0;
volatile uint8_t led_brightness = 0;

volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;

volatile uint8_t offset_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//复制中断向量表到SRAM首地址
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //重映射 SRAM 地址到 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_9 | GPIO_PIN_10);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    gap_value.f = 400.0;

    flash_data[0] = I2C_ADDRESS;
    flash_data[1] = 0;
    flash_data[2] = gap_value.c[0];
    flash_data[3] = gap_value.c[1];
    flash_data[4] = gap_value.c[2];
    flash_data[5] = gap_value.c[3];
    flash_data[6] = 0;
    flash_data[7] = 0;
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    gap_value.c[0] = flash_data[2];
    gap_value.c[1] = flash_data[3];
    gap_value.c[2] = flash_data[4];
    gap_value.c[3] = flash_data[5];
  }
}

void flash_data_write_back(void)
{
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[0] = i2c_address[0];
    flash_data[2] = gap_value.c[0];
    flash_data[3] = gap_value.c[1];
    flash_data[4] = gap_value.c[2];
    flash_data[5] = gap_value.c[3];
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  }     
}

uint32_t HX711_readData(void)
{
	uint8_t time_count = 0;
	volatile uint8_t unused = 0;
	uint32_t count = 0;

	// wait hx711 cover finish
	while (HX711_getDataValue())
	{
		HAL_Delay(1);
		time_count++;
		if (time_count == 200)
		{
			return 0;
		}
	}

  __disable_irq();
	for (uint8_t i = 0; i < 24; i++)
	{
		HX711_setClkValue_High;
		count = count << 1;
		HX711_setClkValue_Low;

		count = count + HX711_getDataValue();
	}

	HX711_setClkValue_High;
	unused = unused + 1;
	HX711_setClkValue_Low;

	count = count ^ 0x800000;
  __enable_irq();
	// count = count >> 5;
	return count;
}

void Get_Weight()
{
  int32_t adc_delta = 0;
  // int32_t value_buf[SAMPLE_TIME] = {0};
  // int32_t sum = 0;
  // int32_t max,min;

	raw_adc = HX711_readData();

  adc_delta = raw_adc - offset_adc;		//获取净重
  if(adc_delta > 0)			
  {	
    weight_value = ((float)adc_delta/gap_value.f); 	//计算实物的实际重量
  } else {
    adc_delta = -adc_delta;
    weight_value = ((float)adc_delta/gap_value.f); 	//计算实物的实际重量
  }   

  // if (index >= SAMPLE_TIME) {
  //   // for( int j = 0; j < SAMPLE_TIME - 1; j++ )
  //   // {
  //   //     for( int i = 0; i < SAMPLE_TIME - j - 1; i++ )
  //   //     {
  //   //         if( value_buf[i] > value_buf[i + 1] )
  //   //         {
  //   //             temp = value_buf[i];
  //   //             value_buf[i] = value_buf[i + 1];
  //   //             value_buf[i + 1] = temp;
  //   //         }
  //   //     }
  //   // }
  //   for(int n=0;n<SAMPLE_TIME;n++)
  //   {
  //     sum += raw_adc_set[n];
  //   }
  //   max=raw_adc_set[0];
  //   min=raw_adc_set[0];

  //   for(int n=0;n<SAMPLE_TIME;n++)//取最大值、最小值
  //   {
  //       max=(raw_adc_set[n]<max)?max:raw_adc_set[n];    
  //       min=(min<raw_adc_set[n])?min:raw_adc_set[n];
  //   }     

  //   raw_adc = (sum - max - min) / (SAMPLE_TIME-2);    
  //   index = 0;
  //   // raw_adc = value_buf[( SAMPLE_TIME - 1 ) / 2];  

  //   adc_delta = raw_adc - offset_adc;		//获取净重
  //   if(adc_delta > 0)			
  //   {	
  //     weight_value = ((float)adc_delta/gap_value.f); 	//计算实物的实际重量
  //   } else {
  //     adc_delta = -adc_delta;
  //     weight_value = ((float)adc_delta/gap_value.f); 	//计算实物的实际重量
  //   }     
  // }
}

void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x0000020B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = i2c_address[0]<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(i2c_address[0]);
  /* USER CODE END I2C1_Init 2 */

}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  uint8_t rx_mark[16] = {0};

  if (len == 1) {
    if (rx_data[0] <= 0x03) {
      i2c1_set_send_data((uint8_t *)&raw_adc, 4);
    }
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x13) {
      i2c1_set_send_data((uint8_t *)&weight_value, 4);
    }
    else if (rx_data[0] == 0x20) {
      uint8_t btn_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
      i2c1_set_send_data((uint8_t *)&btn_value, 1);
    }
    else if (rx_data[0] >= 0x30 && rx_data[0] <= 0x32) {
      tx_buf[0] = led_r;
      tx_buf[1] = led_g;
      tx_buf[2] = led_b;
      i2c1_set_send_data((uint8_t *)&tx_buf, 3);
    }
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      i2c1_set_send_data((uint8_t *)&gap_value.f, 4);
    } 
    else if (rx_data[0] == 0xFF)
    {
      i2c1_set_send_data(i2c_address, 1); 
    }  
    else if (rx_data[0] == 0xFE)
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    }             
  }
  else if (len > 1) {
    if (rx_data[0] >= 0x30 && rx_data[0] <= 0x32) {
      if (len <= 4) {
        for (int i = 0; i < len - 1; i++) {
          rx_buf[rx_data[0]-0x30+i] = rx_data[1+i];
          rx_mark[rx_data[0]-0x30+i] = 1; 
        }
        if (rx_mark[0]) {
          led_r = rx_buf[0];
        }
        if (rx_mark[1]) {
          led_g = rx_buf[1];
        }
        if (rx_mark[2]) {
          led_b = rx_buf[2];
        }
        neopixel_set_color(0, (led_r << 24) | (led_g << 16) | (led_b << 8));
        neopixel_show();
      }
    }
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      if (len <= 5) {
        for (int i = 0; i < len - 1; i++) {
          rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
          rx_mark[rx_data[0]-0x40+i] = 1; 
        }
        if (rx_mark[0]) {
          gap_value.c[0] = rx_buf[0];
        }
        if (rx_mark[1]) {
          gap_value.c[1] = rx_buf[1];
        }
        if (rx_mark[2]) {
          gap_value.c[2] = rx_buf[2];
        }
        if (rx_mark[3]) {
          gap_value.c[3] = rx_buf[3];
        }
        flash_data_write_back();
      }
    }
    else if (rx_data[0] == 0xFD) 
    {
      if (rx_data[1] == 1) {
        flag_jump_bootloader = 1;
        if (flag_jump_bootloader) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);
          i2c_port_set_to_input();
          while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
          {
            jump_bootloader_timeout++;
            if (jump_bootloader_timeout >= 60000) {
              flag_jump_bootloader = 0;
              break;
            }
          }
          if (jump_bootloader_timeout < 60000) {
            NVIC_SystemReset();
          } else {
            user_i2c_init();
            i2c1_it_enable();      
            jump_bootloader_timeout = 0;
          }
        }        
      }
    }
    else if (rx_data[0] == 0xFF)
    {
      if (len == 2) {
        if (rx_data[1] < 128) {
          i2c_address[0] = rx_data[1];
          flash_data_write_back();
          user_i2c_init();
        }
      }
    }          
    else if (rx_data[0] == 0x50)
    {
      if (len == 2) {
        if (rx_data[1]) {
          offset_flag = 1;
        }
      }
    }          
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
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
  // MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  sk6812_init(TOTAL_RGB);
  init_flash_data();
  user_i2c_init();
  HAL_Delay(1000);
  offset_adc = HX711_readData();
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (offset_flag) {
      offset_adc = HX711_readData();
      offset_flag = 0;
    }
    Get_Weight();
    HAL_Delay(100);
    if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
      offset_flag = 1;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

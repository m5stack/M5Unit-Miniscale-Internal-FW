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
#include <stdio.h>
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
#define FIRMWARE_VERSION 4
#define I2C_ADDRESS 0x26
#define FLASH_DATA_SIZE 8
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000) 

#define SAMPLE_TIME 11
#define MAX_RECORD_SIZE 256

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

volatile uint8_t weight_init_flag = 0;
volatile int32_t offset_adc = 0;
volatile int32_t raw_adc = 0;
volatile int32_t raw_adc_set[SAMPLE_TIME] = {0};
volatile float weight_value = 0;
float weight_record[MAX_RECORD_SIZE] = {0};
uint8_t record_index = 0;

volatile uint8_t lp_filter_enabled = 1;
volatile uint8_t avg_filter_level = 10;
volatile float ema_filter_alpha = 10.0f;
volatile uint8_t ema_filter_int = 10;

volatile uint8_t led_r = 0;
volatile uint8_t led_g = 0;
volatile uint8_t led_b = 0;
volatile uint8_t led_brightness = 0;

volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;

volatile uint8_t offset_flag = 0;
volatile uint32_t get_adc_delay = 0;
volatile uint32_t get_data_delay = 0;
volatile uint8_t get_data_flag = 0;

// i2c stop timeout delay
static volatile uint32_t i2c_stop_timeout_delay = 0;

volatile uint8_t sensor_polarity = 0;
char s[16]={"\0"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Filter Order:4.0
//Filter cutoff frequency (-3dB):1.0Hz
//Filter sampling frequency:10.0Hz


//Filter Coefficients
const uint8_t LP_a_bytes[16]={
	0x83,0x38,0x86,0xbf,
	0xb5,0x9f,0x97,0x3e,
	0xb1,0x13,0xa9,0xbf,
	0x2b,0xfb,0x21,0x3f};
const float* LP_a = (float*) LP_a_bytes;

const uint8_t LP_b_bytes[24]={0x86,0x15,0x9e,0x3b,
	0x86,0x15,0x1e,0x3c,
	0x87,0x15,0x9e,0x3b,
	0x0,0x0,0x80,0x3f,
	0x0,0x0,0x0,0x40,
	0x0,0x0,0x80,0x3f};
const float* LP_b = (float*) LP_b_bytes;

//Delay Variables
float LP_w[4]={0,0,0,0};

float LP_Update(float x)
{
	float y;
	float w;
	//Section 0
	w = x-LP_w[0]*LP_a[0]-LP_w[1]*LP_a[1];
	y = w*LP_b[0]+LP_w[0]*LP_b[1]+LP_w[1]*LP_b[2];
	LP_w[1]=LP_w[0];
	LP_w[0]=w;
	//Section 1
	w = y-LP_w[2]*LP_a[2]-LP_w[3]*LP_a[3];
	y = w*LP_b[3]+LP_w[2]*LP_b[4]+LP_w[3]*LP_b[5];
	LP_w[3]=LP_w[2];
	LP_w[2]=w;
	return y;
}

float ema_filter(float ema_filtered, float new_val, float alpha)
{
    ema_filtered = (1 - alpha) * ema_filtered + alpha * new_val;
    return ema_filtered;
}

float avg_filter(float *data, int len)
{
	float sum = 0;
	float min = data[0];
	float max = data[0];
	for (int i = 0; i < len; i++) {
		if (data[i] < min) {
			min = data[i];
		}
		if (data[i] > max) {
			max = data[i];
		}
    sum += data[i];
	}

	sum -= min;
	sum -= max;

	return sum / (len - 2);
}

void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//�����ж���������SRAM�׵�ַ
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //��ӳ�� SRAM ��ַ�� 0x00000000
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

static void init_flash_data(void) 
{   
  uint32_t flash_write_timeout = 0;

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

    // retry 20 times, if still fail, fast flash 10 times, reset system
    while(!writeMessageToFlash(flash_data , FLASH_DATA_SIZE)) {
      flash_write_timeout++;
      if (flash_write_timeout > 20) {        
        HAL_NVIC_SystemReset();
      }
    }
  } else {
    i2c_address[0] = flash_data[0];
    gap_value.c[0] = flash_data[2];
    gap_value.c[1] = flash_data[3];
    gap_value.c[2] = flash_data[4];
    gap_value.c[3] = flash_data[5];
  }
}

uint8_t flash_data_write_back(void)
{
  uint32_t flash_write_timeout = 0;

  // if read falsh ok
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[0] = i2c_address[0];
    flash_data[2] = gap_value.c[0];
    flash_data[3] = gap_value.c[1];
    flash_data[4] = gap_value.c[2];
    flash_data[5] = gap_value.c[3];
    // retry 20 times, if still fail, return 0
    while(!writeMessageToFlash(flash_data , FLASH_DATA_SIZE)) {
      flash_write_timeout++;
      if (flash_write_timeout > 20) {
        flash_write_timeout = 0;
        return 0;
      }
    }
    // write success, return 1
    return 1;
  }
  else {
    return 0;
  }     
}

void rainbowLED(void)
{
#define LEDSPEED 0xf
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blue = 0;
	uint32_t tempColor = 0;
  uint8_t t = 0;

	while (t <= (LEDSPEED * 6))
	{
		t++;
		if (t < LEDSPEED)
		{
			red = 0xff;
			blue = 0x0;
			green += (0xff / LEDSPEED);
		}
		if (t > LEDSPEED && t < LEDSPEED * 2)
		{
			red -= (0xff / LEDSPEED);
			blue = 0x0;
			green = 0xff;
		}
		if (t > LEDSPEED * 2 && t < LEDSPEED * 3)
		{
			red = 0x0;
			blue += (0xff / LEDSPEED);
			green = 0xff;
		}
		if (t > LEDSPEED * 3 && t < LEDSPEED * 4)
		{
			red = 0x0;
			blue = 0xff;
			green -= (0xff / LEDSPEED);
		}
		if (t > LEDSPEED * 4 && t < LEDSPEED * 5)
		{
			red += (0xff / LEDSPEED);
			blue = 0xff;
			green = 0x0;
		}
		if (t > LEDSPEED * 5 && t < LEDSPEED * 6)
		{
			red -= (0xff / LEDSPEED);
			blue -= (0xff / LEDSPEED);
			green = 0x0;
		}
		if (t == LEDSPEED * 6)
		{
			red = 0x0;
			blue = 0x0;
			green = 0x0;
		}
		tempColor = (red) | (green << 8) | (blue << 16);
		neopixel_set_color(0, tempColor);
		neopixel_show();
		HAL_Delay(5);
		tempColor = 0x00;
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
	return count;
}

void Get_Weight()
{
  int32_t adc_delta = 0;

	raw_adc = HX711_readData();

  adc_delta = raw_adc - offset_adc;		
  if (gap_value.f >= -0.05f && gap_value.f <= 0.05f)
    gap_value.f = 1.0f;
  if(sensor_polarity == 0)			
  {	
    weight_value = ((float)adc_delta/gap_value.f); 	
  } else {
    adc_delta = -adc_delta;
    weight_value = ((float)adc_delta/gap_value.f); 	
  }   

  if (lp_filter_enabled) {
    weight_value = LP_Update(weight_value);
  }

  if (avg_filter_level != 0) {
    weight_record[record_index] = weight_value;
    record_index++;
    if (record_index >= avg_filter_level) {
      record_index = 0;
    }
    weight_value = avg_filter(weight_record, avg_filter_level);
  }
  else if (ema_filter_alpha != 0) {
    weight_value = ema_filter(weight_record[0], weight_value, ema_filter_alpha);
    weight_record[0] = weight_value;
  }  
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
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      i2c1_set_send_data((uint8_t *)&gap_value.f, 4);
    } 
    else if (rx_data[0] >= 0x60 && rx_data[0] <= 0x63) {
      int32_t weight_int = weight_value * 100;
      i2c1_set_send_data((uint8_t *)&weight_int, 4);
    } 
    else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x7F) {
      i2c1_set_send_data((uint8_t *)&s, 16);
    }    
    else if (rx_data[0] == 0xFF)
    {
      i2c1_set_send_data(i2c_address, 1); 
    }  
    else if (rx_data[0] == 0xFE)
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    }     
    else if (rx_data[0] == 0x90)
    {
      i2c1_set_send_data((uint8_t *)&sensor_polarity, 1);
    }     
    else if (rx_data[0] >= 0x80 && rx_data[0] <= 0x82)
    {
      tx_buf[0] = lp_filter_enabled;
      tx_buf[1] = avg_filter_level;
      tx_buf[2] = ema_filter_int;   
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x80], 0x82-rx_data[0]+1);   
    }             
  }
  else if (len > 1) {
    if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
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
          NVIC_SystemReset();
        }        
      }
    }
    else if (rx_data[0] == 0xFF)
    {
      if (len == 2) {
        if (rx_data[1] && (rx_data[1] < 128)) {
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
    else if (rx_data[0] >= 0x80 && rx_data[0] <= 0x82)
    {
      for (int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x80+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x80+i] = 1; 
      }
      if (rx_mark[0]) {
        lp_filter_enabled = !!rx_buf[0];
      }
      if (rx_mark[1]) {
        avg_filter_level = rx_buf[1];
        record_index = 0;
      }
      if (rx_mark[2]) {
        if (rx_buf[2]) {
          ema_filter_int = rx_buf[2];
          ema_filter_alpha = (float)rx_buf[2] / 100.0f;
        }
        else {
          ema_filter_int = 0;
          ema_filter_alpha = 0.0f;
        }
      }
    }
    else if (rx_data[0] == 0x90)
    {
      if (len == 2) {
        sensor_polarity = rx_data[1];
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
  /* USER CODE BEGIN 2 */
  init_flash_data();
  user_i2c_init();
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (get_adc_delay < HAL_GetTick()) {
      Get_Weight();
      snprintf(s, 15, "%.2f", weight_value);
      get_adc_delay = HAL_GetTick() + 90;
    }  

    if (!weight_init_flag) {
      offset_adc = raw_adc;
      weight_init_flag = 1;
    }
    i2c_timeout_counter = 0;
    if (offset_flag) {
      offset_adc = raw_adc;
      offset_flag = 0;
    }
    i2c_timeout_counter = 0;
    if (i2c_stop_timeout_flag) {
      if (i2c_stop_timeout_delay < HAL_GetTick()) {
        i2c_stop_timeout_counter++;
        i2c_stop_timeout_delay = HAL_GetTick() + 10;
      }
    }

    if (i2c_stop_timeout_counter > 50) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);     
      user_i2c_init();    
      i2c1_it_enable();
      HAL_Delay(500);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
    HAL_NVIC_SystemReset();
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

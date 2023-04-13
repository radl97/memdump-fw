/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

state st = run;
uint8_t cmd_buf [BUFSIZ] = {0};
uint8_t cmd_pos = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -----------------------------------------------------------------------------
 * Converts a hexadecimal ASCII character to it's value
 *
 * From https://github.com/Sasszem/oldschool-game/blob/main/save.c
 * under the conditions of the MIT License:
 *
 * MIT License
 *
 * Copyright (c) 2022 László Baráth
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * -------------------------------------------------------------------------- */
uint8_t hex_nibble(const char val) {
  if ('0' <= val && val <= '9')
    return val - '0';
  if ('a' <= val && val <= 'f')
    return val - 'a' + 10;
  if ('A' <= val && val <= 'F')
    return val - 'A' + 10;
  return 0xFF;
}

/* -----------------------------------------------------------------------------------------
 * Converts all characters until the first character that matches delim or until *Len
 * Result is stored to dest
 * Return value is the number of characters processed or 0 in case of a conversion failure
 * ----------------------------------------------------------------------------------------- */
uint8_t homegrown_scanf (uint8_t *Buf, uint8_t Len, uint32_t *dest, const char delim) {
  uint8_t cursor = 0;
  for (*dest = 0; Buf[cursor] != delim && cursor < Len; cursor++) {
    uint8_t nibble = hex_nibble(Buf[cursor]);
    if (nibble == 0xFF) {
      *dest = 0;
      return(cursor + 1);
    }
    *dest = *dest * 16 + nibble;
  }
  return (cursor + 1);
}
/* --------------------------------------------------------------------------
 * convert a hex to ascii string
 * input is a byte array (binary), output is a character array (str)
 * len is the number of bytes to convert, str needs to be at least 2x as big
 * -------------------------------------------------------------------------- */
void hex_to_ascii (const uint8_t *binary, char *str, uint16_t len) {
  for (uint16_t i = 0; i < len; ++i) {
    uint8_t low = binary [i] & 0x0f;
    uint8_t high = binary [i] >> 4;
    str [2 * i] = (low <= 0x9) ? low + '0' : low - 0xA + 'A';
    str [2 * i + 1] = (high <= 0x9) ? high + '0' : high - 0xA + 'A';
  }
}

char *hal_reason[] = {
  "OK     \n\r", // HAL_OK == 0
  "ERROR  \n\r", // HAL_ERROR == 1
  "BUSY   \n\r", // HAL_BUSY == 2
  "TIMEOUT\n\r", // HAL_TIMEOUT == 3
};

void i2c_dump(uint16_t dev_address, uint16_t capacity) {
  char tx_buff [BUFSIZ] = {0};
  uint8_t read_buff [BUFSIZ / 2] = {0};
  for (uint16_t i = 0; i < capacity; i += BUFSIZ / 2) {
    uint8_t reason;
    if ((reason = HAL_I2C_Mem_Read(&hi2c1, dev_address << 1, i, capacity, read_buff, BUFSIZ/2, 1000)) != HAL_OK) {

      CDC_Transmit_FS((uint8_t *) "No memory on address\n\r", 22);
      HAL_Delay(100);
      CDC_Transmit_FS((uint8_t *) hal_reason[reason], 9); // Got ERROR, and hi2c1.ErrorCode == HAL_I2C_ERROR_TIMEOUT
      HAL_Delay(100);

      __HAL_RCC_I2C1_FORCE_RESET();
      HAL_Delay(100);
      __HAL_RCC_I2C1_RELEASE_RESET();
    }
    else {
      hex_to_ascii(read_buff, tx_buff, BUFSIZ/2);
      CDC_Transmit_FS((uint8_t *) tx_buff, BUFSIZ);
      CDC_Transmit_FS((uint8_t *) "\n\r", 2);
    }
  }
}

// todo spi, d28, d32 read & write, i2c write

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  char error_msg[] = "Invalid command!\n\r";
  uint32_t i2c_address = 0;
  uint32_t limit_address = 0;
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(100);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_GPIO_WritePin(ARST_GPIO_Port, ARST_Pin, 0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(ARST_GPIO_Port, ARST_Pin, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    HAL_Delay(100);
    //if received \r start decoding state
    if (cmd_buf[cmd_pos - 1] == '\r') {
      // receive the command, set state
      if (cmd_buf[0] == 'r') {
        switch (cmd_buf[1]) {
          case 'i':
            st = read_i2c;
            break;
          case 's':
            st = read_spi;
            break;
          case '2':
            st = read_d28;
            break;
          case '3':
            st = read_d32;
            break;
          default:
            st = invalid;
        }
      }
      else if (cmd_buf[0] == 'w') {
        switch (cmd_buf[1]) {
          case 'i':
            st = write_i2c;
            break;
          case 's':
            st = write_spi;
            break;
          case '2':
            st = write_d28;
            break;
          case '3':
            st = write_d32;
            break;
          default:
            st = invalid;
        }
      }
      else {
        st = invalid;
      }
      // receive optional i2c address and limit address
      uint8_t cursor = 3;                                                                     // 3 = 2 for the command, 1 for a separator
      cmd_pos -= cursor - 1;
      if (st == write_i2c || st == read_i2c) {
        cursor += (homegrown_scanf(cmd_buf + cursor, cmd_pos, &i2c_address, ' '));
        if (i2c_address == 0) {
          st = invalid;
        }
        cmd_pos -= cursor - 1;
      }
      homegrown_scanf(cmd_buf + cursor, cmd_pos, &limit_address, '\r');
      if (limit_address == 0) {
        st = invalid;
      }
      //reset cmd_buf and cmd_pos
      memset(cmd_buf, 0, BUFSIZ);
      cmd_pos = 0;
    }
    switch (st) {
      case read_i2c:
        i2c_dump(i2c_address, limit_address);
        i2c_address = 0;
        limit_address = 0;
        st = run;
        break;
      case read_spi:
        limit_address = 0;
        st = run;
        break;
      case read_d28:
        limit_address = 0;
        st = run;
        break;
      case read_d32:
        limit_address = 0;
        st = run;
        break;
      case write_i2c:
        limit_address = 0;
        st = run;
        break;
      case write_spi:
        limit_address = 0;
        st = run;
        break;
      case write_d28:
        limit_address = 0;
        st = run;
        break;
      case write_d32:
        limit_address = 0;
        st = run;
        break;
      case invalid:
        CDC_Transmit_FS( (uint8_t *) error_msg, strlen(error_msg));
        st = run;
      default:
        continue;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACLK_GPIO_Port, ACLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |SZE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ARST_Pin ADATA_Pin */
  GPIO_InitStruct.Pin = ARST_Pin|ADATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACLK_Pin */
  GPIO_InitStruct.Pin = ACLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin
                           D4_Pin D5_Pin D6_Pin D7_Pin
                           SZE_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |SZE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WP_Pin NCE_Pin NWE_Pin */
  GPIO_InitStruct.Pin = WP_Pin|NCE_Pin|NWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

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
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "stm32f302x8.h"
#include "stm32f3xx_hal_dma.h"
#include "tim.h"
#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

/* TIM2 masterclock,TIM1 PWMs, USART2 communication*/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float Kp;          // factor proporcional
  float Ki;          // factor integral
  float Kd;          // factor  derivativo
  float error;       // error
  int32_t threshold; // banda muerta (evitara las vibraciones cuando el error
                     // sea menor al umbral)
  uint32_t pwm_min;  // pwm minimo para romper la inercia y friccion estatica
  uint32_t
      pwm_max; // evitara que el motor toque limites fisicos a maxima potencia
  int32_t top_limit;    // limitando el giro hacia posicion positiva para que no
                        // toque los limimites fisicos
  int32_t botton_limit; // limitando el giro hacia posicion negativa para que no
                        // toque los limimites fisicos
  int32_t anti_windup;  // evitara el windup del integrador
} Config_Pid_t;

typedef struct {
  int32_t position; // posicion actual del motor
  int32_t setpoint; // posicion deseada del motor
  float Kp;         // factor proporcional
  float Ki;         // factor integral
  float Kd;         // factor derivativo
  int32_t output;   // valor de salida del PID (pwm)

} MotorHandle_typedf_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flag_execute_pid = 0; // bandera para ejecutar el PID en el timer
uint16_t Num_motors_values_received =
    0; // contador de los valores recibidos por UART, mimimo 6 valores para
       // ejecutar el PID
uint8_t uart1_rx_buffer[9]; // buffer de recepcion para USART1
uint16_t rx_buffer_size;
uint8_t snapshot[9];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  rx_buffer_size = sizeof(uart1_rx_buffer);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer,
                               sizeof(uart1_rx_buffer));
  HAL_TIM_PWM_Init(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  HAL_GPIO_WritePin(INA_a_GPIO_Port, INA_a_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INA_b_GPIO_Port, INA_b_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INB_a_GPIO_Port, INB_a_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INB_b_GPIO_Port, INB_b_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INC_a_GPIO_Port, INC_a_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INC_b_GPIO_Port, INC_b_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    switch (Num_motors_values_received) {
    case 2: { //caso de 2DOF
    }
    case 3: { // caso de 3DOF
    }
    default: {
      /*error handle*/
    }
    }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*################ master clock############################*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    // Handle TIM1 period elapsed event
  }
}

/*################ data receive ############################*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    memcpy(snapshot, uart1_rx_buffer, rx_buffer_size);
    Num_motors_values_received = Size/3;
    flag_execute_pid = 1;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, rx_buffer_size);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx,DMA_IT_HT);
  }

  // Handle USART1 receive complete event
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

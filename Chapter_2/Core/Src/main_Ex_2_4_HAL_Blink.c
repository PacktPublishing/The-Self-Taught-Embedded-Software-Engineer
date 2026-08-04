/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main_Ex_2_4_HAL_Blink.c
 * @author  Renato Soriano
 * @brief   Chapter 2 — Exercise 2.4: Your First Program — Blinking an LED
 *          (HAL + CubeMX-generated skeleton, Nucleo-F446RE)
 *
 * @target  STM32F446RE — Nucleo-F446RE
 * @board   LD2 user LED → PA5 (active HIGH)
 *
 * @details This file represents the application-level code you obtain after
 *          running STM32CubeMX to configure the project and then modifying
 *          the generated main.c to add the blink logic.
 *
 *          CubeMX generates three things you will find in this file:
 *
 *          1. SystemClock_Config() — sets up the MCU clock tree.
 *             For this exercise the defaults (HSI 16 MHz) are sufficient;
 *             CubeMX writes this function automatically.
 *
 *          2. MX_GPIO_Init() — configures PA5 as a push-pull output so that
 *             the LED can be driven. CubeMX generates this from the Pinout
 *             view when you label PA5 as "LD2".
 *
 *          3. The super-loop in main() — the two lines you type yourself:
 *             HAL_GPIO_TogglePin() and HAL_Delay().
 *
 *          What this exercise teaches:
 *          - How CubeMX-generated code is structured (main.c, HAL, system
 *            files) and where user code must be placed.
 *          - What HAL_GPIO_TogglePin() and HAL_Delay() do at a high level.
 *          - How to change the blink rate by modifying the delay argument.
 *          - How to build, flash, and verify a project end-to-end.
 *
 * @note    The USER CODE BEGIN / END markers are the regions where CubeMX
 *          will not overwrite your code if you regenerate the project.
 *          Always place your logic inside these markers.
 *
 * @note    The stubs for SystemClock_Config() and MX_GPIO_Init() are shown
 *          here in condensed, readable form. CubeMX generates fuller
 *          versions with HAL assert macros and error handlers; those
 *          additions are described in the chapter but omitted here for
 *          clarity.
 ******************************************************************************
 */
/* USER CODE END Header */
/* -------------------------------------------------------------------------
 * Includes — CubeMX always adds main.h which pulls in stm32f4xx_hal.h
 * and ultimately the device-specific header (stm32f446xx.h).
 * ------------------------------------------------------------------------- */
#include "main.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 * CubeMX generates the HAL_Init() call, the clock configuration call, and
 * the peripheral-init calls.  You add your application logic in the
 * super-loop between USER CODE BEGIN and USER CODE END.
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick.
	 * Systick is configured to generate a 1 ms tick — HAL_Delay() relies
	 * on this tick counter. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock (HSI 16 MHz with default PLL settings).*/
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* Toggle LD2 (PA5) — flips the pin state each time it is called. */
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

		/* Wait 500 ms.  Try changing this value:
		 *   250  → faster blink (2 Hz)
		 *   1000 → slower blink (0.5 Hz)
		 * The argument is in milliseconds; Systick provides the timebase. */
		HAL_Delay(500U);

	}
	/* USER CODE END 3 */
	return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 * Configures the MCU system clock.  At reset, STM32F446RE runs on the
 * internal HSI oscillator at 16 MHz.  For this exercise that speed is
 * sufficient — CubeMX generates a more complete version when you configure
 * a specific clock frequency in the Clock Configuration view.
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 * Use the internal HSI oscillator at 16 MHz — no crystal needed.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 * Configures PA5 as a push-pull output (no pull resistor, low speed).
 * CubeMX generates this function automatically when you set PA5 to
 * GPIO_Output in the Pinout & Configuration view.
 *
 * Key parameters explained:
 *   Mode  = GPIO_MODE_OUTPUT_PP  → push-pull (drives both HIGH and LOW)
 *   Pull  = GPIO_NOPULL          → no internal resistor (LED has its own)
 *   Speed = GPIO_SPEED_FREQ_LOW  → reduces EMI; sufficient for LED blinking
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable BEFORE writing any GPIOA register */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level, Set the initial output level to LOW (LED off at startup)*/
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 *  Called if any HAL initialisation function returns HAL_ERROR.
 * In this exercise the handler simply loops; in production firmware you
 * would log the error and trigger a safe-state routine.
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		/* Trap here so a debugger can catch the failure                   */
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

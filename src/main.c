#include <stdbool.h>

#include "cmsis_os.h"
#include "main.h"

#include <usbd_core.h>
#include <usbd_cdc.h>
#include "usbd_cdc_interface.h"
#include "usbd_desc.h"

osThreadId miscThreadHandle;

USBD_HandleTypeDef USBD_Device;
extern USBD_DescriptorsTypeDef VCP_Desc;

static void miscThread(void const *argument);

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	while(1) ;
}


#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *	where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/*
	 * User can add his own implementation to report the file name and line number,
	 * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
	 */

	while (1) ;
}
#endif

/**
 * @brief  System Clock Configuration
 *	The system Clock is configured as follow :
 *		System Clock source	= PLL (HSE)
 *		SYSCLK(Hz)		= 168000000
 *		HCLK(Hz)		= 168000000
 *		AHB Prescaler		= 1
 *		APB1 Prescaler		= 4
 *		APB2 Prescaler		= 2
 *		HSE Frequency(Hz)	= 8000000
 *		PLL_M			= 8
 *		PLL_N			= 336
 *		PLL_P			= 2
 *		PLL_Q			= 7
 *		VDD(V)			= 3.3
 *		Main reg. output	= Scale1 mode
 *		Flash Latency(WS)	= 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/*
	 * The voltage scaling allows optimizing the power consumption when the device is
	 * clocked below the maximum system frequency, to update the voltage scaling value
	 * regarding system frequency refer to product datasheet.
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
	if (HAL_GetREVID() == 0x1001) {
		/* Enable the Flash prefetch */
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	}
}


#if 0
void HAL_MspInit(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#endif


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		GPIO_InitTypeDef gpio;

		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/*
		 * TIM2 GPIO Configuration
		 * PA0 - TIM2_CH1
		 * PA1 - TIM2_CH2
		 * PA2 - TIM2_CH3
		 */
		memset(&gpio, 0, sizeof(gpio));
		gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
		gpio.Mode = GPIO_MODE_OUTPUT_PP;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_HIGH;
		gpio.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &gpio);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	}
}


void gpio_init(void)
{
	GPIO_InitTypeDef gpio;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	memset(&gpio, 0, sizeof(gpio));

	/* PA3 = M1DIR */
	gpio.Pin = GPIO_PIN_3;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &gpio);

	/* PC13..15 = PWM enables */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

	/* PC4..5 = M2/M3DIR, PC13..15 = PWM enables */
	gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOC, &gpio);

	/* PD10..11 == LED1/2 */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

	gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	HAL_GPIO_Init(GPIOD, &gpio);
}


void pwm_set(TIM_HandleTypeDef *htim, uint32_t which, uint16_t val)
{
	TIM_OC_InitTypeDef oc;

	memset(&oc, 0, sizeof(oc));

	oc.OCMode = TIM_OCMODE_PWM1;
	oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc.OCFastMode = TIM_OCFAST_DISABLE;

	oc.Pulse = val;

	HAL_TIM_PWM_ConfigChannel(htim, &oc, which);
}


void pwm_init(TIM_HandleTypeDef *htim)
{
	TIM_ClockConfigTypeDef tclkcfg;
	TIM_MasterConfigTypeDef tmcfg;

	memset(htim, 0, sizeof(*htim));

	htim->Instance = TIM2;
	htim->Init.Prescaler = 24;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = 2000;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(htim);

#if 1
	memset(&tclkcfg, 0, sizeof(tclkcfg));
	memset(&tmcfg, 0, sizeof(tmcfg));

	tclkcfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(htim, &tclkcfg);

	tmcfg.MasterOutputTrigger = TIM_TRGO_RESET;
	tmcfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(htim, &tmcfg);
#endif

	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

	pwm_set(htim, TIM_CHANNEL_1, 1500);
	pwm_set(htim, TIM_CHANNEL_2, 700);
	pwm_set(htim, TIM_CHANNEL_3, 250);
}


void m1dir(bool val) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (val) ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void m2dir(bool val) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, (val) ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void m3dir(bool val) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (val) ? GPIO_PIN_SET : GPIO_PIN_RESET); }


static void miscThread(void const *argument)
{
	TIM_HandleTypeDef htim;
	int x;

	(void)argument;

	usb_init();
	gpio_init();
	pwm_init(&htim);

	m1dir(false);
	m2dir(false);
	m3dir(false);

	x = 0;
	while (1) {
		bool toggle;
		osDelay(10);

//		pwm_set(&htim, TIM_CHANNEL_1, x);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

		++x;
		if (x >= 2000) {
			x = 0;
		}


		/* blinky */
		if ((x % 10) == 0) {
			if (toggle) {
				toggle = false;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			} else {
				toggle = true;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			}
		}

		if ((x % 250) == 0) {
			printf("Foo (%d)\n", osKernelSysTick());
		}
	};
}


void usb_init(void)
{
	USBD_Init(&USBD_Device, &VCP_Desc, 0);
	USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
	USBD_Start(&USBD_Device);
}


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	osThreadDef(MISC, miscThread, osPriorityNormal, 0, 1024);

	miscThreadHandle = osThreadCreate(osThread(MISC), NULL);
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	while (1) ;
}

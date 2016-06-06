#include <cmsis_os.h>
#include "stm32f4xx_hal.h"

/* from http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html */
static __attribute__((used)) void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
	/*
	 * These are volatile to try and prevent the compiler/linker optimising them
	 * away as the variables never actually get used.  If the debugger won't show the
	 * values of the variables, make them global my moving their declaration outside
	 * of this function.
	 */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	/* prevent compiler warnings about set-but-unused variables */
	(void)r0;
	(void)r1;
	(void)r2;
	(void)r3;
	(void)r12;
	(void)lr;
	(void)pc;
	(void)psr;

	while (1) ;
}


void NMI_Handler(void)
{
	while (1) ;
}


void HardFault_Handler(void)
{
#if 1
	__asm volatile (
		" tst lr, #4                                                \n"
		" ite eq                                                    \n"
		" mrseq r0, msp                                             \n"
		" mrsne r0, psp                                             \n"
		" ldr r1, [r0, #24]                                         \n"
		" ldr r2, handler2_address_const                            \n"
		" bx r2                                                     \n"
		" handler2_address_const: .word prvGetRegistersFromStack    \n"
	);
#endif

	while (1) ;
}

void MemManage_Handler(void)
{
	while (1) ;
}

void BusFault_Handler(void)
{
	while (1) ;
}

void UsageFault_Handler(void)
{
	while (1) ;
}

/* SVC_Handler() handled by FreeRTOS */

void DebugMon_Handler(void)
{
}

/* PendSV_Handler() handled by FreeRTOS */

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

void OTG_HS_IRQHandler(void)
{
	 extern PCD_HandleTypeDef hpcd;
	 HAL_PCD_IRQHandler(&hpcd);
}

void TIM4_IRQHandler(void)
{
	extern TIM_HandleTypeDef hcdc_timer;
	HAL_TIM_IRQHandler(&hcdc_timer);
}

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

static struct uart uart1;
static struct uart uart2;
static struct uart uart3;
static struct uart uart4;
static struct uart uart6;
struct uart cdc;

static inline bool locked(volatile int *lock)
{
	return false;
	if (*lock == 0) {
		return false;
	} else {
		return true;
	}
}


static inline void unlock(volatile int *lock)
{
	*lock = 0;
}


static inline bool trylock(volatile int *lock)
{
	if (locked(lock)) {
		return false;
	} else {
		*lock = 1;
		return true;
	}
}


static void fifo_reset(struct fifo *fifo)
{
	fifo->head = fifo->tail = &fifo->data[0];
	unlock(&fifo->lock);
}


/* attempts to write the next value to the FIFO. Returns true if the value was written. */
bool fifo_put(struct fifo *fifo, const char val)
{
	char *next;
	bool ret;

	if (trylock(&fifo->lock)) {
		/* calculate next head position to determine if the value will fit */
		next = (char *)fifo->head;
		++next;
		if (next == &fifo->data[FIFO_LENGTH]) {
			next = (char *)&fifo->data[0];
		}

		/* if the FIFO isn't full (next == tail), write the value to the FIFO and update head. */
		if (next != fifo->tail) {
			*fifo->head = val;
			fifo->head = next;
			ret = true;
		} else {
			ret = false;
		}

		unlock(&fifo->lock);

	/* couldn't get lock */
	} else {
		ret = false;
	}

	return ret;
}


bool fifo_get(struct fifo *fifo, char *dest)
{
	bool ret;

	if (trylock(&fifo->lock)) {
		if (fifo->head != fifo->tail) {
			uint8_t val;

			val = *fifo->tail;

			*fifo->tail++;
			if (fifo->tail == &fifo->data[FIFO_LENGTH]) {
				fifo->tail = &fifo->data[0];
			}

			*dest = val;
			ret = true;
		} else {
			ret = false;
		}

		unlock(&fifo->lock);

	} else {
		ret = false;
	}

	return ret;
}


static void generic_usart_handler(struct uart *u)
{
	UART_HandleTypeDef *huart;
	uint32_t tmp1, tmp2;
	bool err;

	err = false;
	huart = u->huart;

	if (! locked(&u->rx.lock)) {
		/* UART parity error interrupt occurred ------------------------------------*/
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);
		if ((tmp1 != RESET) && (tmp2 != RESET)) {
			__HAL_UART_CLEAR_PEFLAG(huart);
			err = true;
		}

		/* UART frame error interrupt occurred -------------------------------------*/
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
		if ((tmp1 != RESET) && (tmp2 != RESET)) {
			__HAL_UART_CLEAR_FEFLAG(huart);
			err = true;
		}

		/* UART noise error interrupt occurred -------------------------------------*/
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
		if ((tmp1 != RESET) && (tmp2 != RESET)) {
			__HAL_UART_CLEAR_NEFLAG(huart);
			err = true;
		}

		/* UART Over-Run interrupt occurred ----------------------------------------*/
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
		if ((tmp1 != RESET) && (tmp2 != RESET)) {
			__HAL_UART_CLEAR_OREFLAG(huart);
			err = true;
		}

		/* UART in mode Receiver ---------------------------------------------------*/
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
		if((tmp1 != RESET) && (tmp2 != RESET)) {
			uint16_t val;

			val = (uint16_t)(huart->Instance->DR);

			/* don't put errored data into the FIFO */
			if (!err) {
				fifo_put(&u->rx, val);
			}
		}
	}

	if (! locked(&u->tx.lock)) {
		/* UART in mode Transmitter ------------------------------------------------*/
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
		if((tmp1 != RESET) && (tmp2 != RESET)) {
			char val;

			/*
			 * if there's data to send, send it.
			 * otherwise disable the transmit empty interrupt.
			 */
			if (fifo_get(&u->tx, &val)) {
				huart->Instance->DR = val;
			} else {
				__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
			}
		}
	}

#if 0
	/* UART in mode Transmitter end --------------------------------------------*/
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TC);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
	if((tmp1 != RESET) && (tmp2 != RESET)) {
		uint16_t val;

		val = (uint16_t)(huart->Instance->DR);
	}
#endif
}


/* the actual ISRs just call the generic UART handler */
void USART1_IRQHandler(void)
{
	generic_usart_handler(&uart1);
}


void USART2_IRQHandler(void)
{
	generic_usart_handler(&uart2);
}


void USART3_IRQHandler(void)
{
	generic_usart_handler(&uart3);
}


void UART4_IRQHandler(void)
{
	generic_usart_handler(&uart4);
}


struct uart *uart_hw_init(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	struct uart *u;

	switch ((int)huart->Instance) {
	case (int)USART1:
		/* enable GPIO and UART clocks. */
		__HAL_RCC_GPIOB_CLK_ENABLE();		/* TX on PORTB bit 6/7 */
		__HAL_RCC_USART1_CLK_ENABLE();

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* NVIC for USARTx */
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART1_IRQn);

		u = &uart1;
		break;

	case (int)USART2:
		/* enable GPIO and UART clocks. */
		__HAL_RCC_GPIOD_CLK_ENABLE();		/* TX/RX both on PORTD, bits 5 and 6 */
		__HAL_RCC_GPIOB_CLK_ENABLE();		/* DE both on PORTB, bits 4 */
		__HAL_RCC_USART2_CLK_ENABLE();

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* Configure PB4 pin as output (USART2 DE) */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* NVIC for USARTx */
		HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(USART2_IRQn);

		u = &uart2;
		break;

	case (int)USART3:
		/* enable GPIO and UART clocks. */
		__HAL_RCC_GPIOD_CLK_ENABLE();		/* TX/RX on PORTD bit 8/9 */
		__HAL_RCC_USART3_CLK_ENABLE();

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* NVIC for USARTx */
		HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(USART3_IRQn);

		u = &uart3;
		break;

	case (int)UART4:
		/* enable GPIO and UART clocks. */
		__HAL_RCC_GPIOA_CLK_ENABLE();		/* TX/RX both on PORTA bits 0 and 1 */
		__HAL_RCC_UART4_CLK_ENABLE();

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* NVIC for USARTx */
		HAL_NVIC_SetPriority(UART4_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(UART4_IRQn);

		u = &uart4;
		break;

	default:
		/* do nothing */
		u = NULL;
		break;
	};

	/* initialize the uart's FIFO subsystem */
	if (u) {
		u->huart = huart;
		fifo_reset(&u->tx);
		fifo_reset(&u->rx);

		/* enable UART RX interrupt. Disable the TX interrupt since the FIFO's empty right now anyway */
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);		/* receiver not empty */
		__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);		/* transmit empty */
	}

	return u;
}


static void uart_setconfig(UART_HandleTypeDef *huart)
{
	uint32_t tmpreg = 0x00;

	/* Check the parameters */
	assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
	assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
	assert_param(IS_UART_PARITY(huart->Init.Parity));
	assert_param(IS_UART_MODE(huart->Init.Mode));

	/*-------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = huart->Instance->CR2;

	/* Clear STOP[13:12] bits */
	tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

	/* Configure the UART Stop Bits: Set STOP[13:12] bits according to huart->Init.StopBits value */
	tmpreg |= (uint32_t)huart->Init.StopBits;

	/* Write to USART CR2 */
	huart->Instance->CR2 = (uint32_t)tmpreg;

	/*-------------------------- USART CR1 Configuration -----------------------*/
	tmpreg = huart->Instance->CR1;

	/* Clear M, PCE, PS, TE and RE bits */
	tmpreg &= (uint32_t)~((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8));

	/*
	 * Configure the UART Word Length, Parity and mode:
	 * Set the M bits according to huart->Init.WordLength value
	 * Set PCE and PS bits according to huart->Init.Parity value
	 * Set TE and RE bits according to huart->Init.Mode value
	 * Set OVER8 bit according to huart->Init.OverSampling value
	 */
	tmpreg |= (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling;

	/* Write to USART CR1 */
	huart->Instance->CR1 = (uint32_t)tmpreg;

	/*-------------------------- USART CR3 Configuration -----------------------*/
	tmpreg = huart->Instance->CR3;

	/* Clear CTSE and RTSE bits */
	tmpreg &= (uint32_t)~((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE));

	/* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
	tmpreg |= huart->Init.HwFlowCtl;

	/* Write to USART CR3 */
	huart->Instance->CR3 = (uint32_t)tmpreg;

	/* Check the Over Sampling */
	if (huart->Init.OverSampling == UART_OVERSAMPLING_8) {
		/*-------------------------- USART BRR Configuration ---------------------*/
		if ((huart->Instance == USART1) || (huart->Instance == USART6)) {
			huart->Instance->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
		} else {
			huart->Instance->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
		}
	} else {
		/*-------------------------- USART BRR Configuration ---------------------*/
		if ((huart->Instance == USART1) || (huart->Instance == USART6)) {
			huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
		} else {
			huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
		}
	}
}


struct uart *uart_init(UART_HandleTypeDef *huart)
{
	struct uart *u;

	/* Check the parameters */
	if(huart->Init.HwFlowCtl != UART_HWCONTROL_NONE) {
		/* The hardware flow control is available only for USART1, USART2, USART3 and USART6 */
		assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
		assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
	} else {
		assert_param(IS_UART_INSTANCE(huart->Instance));
	}

	assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
	assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

/* always initialize the hardware */
//	if(huart->State == HAL_UART_STATE_RESET) {
		/* Allocate lock resource and initialize it */
		huart->Lock = HAL_UNLOCKED;

		/* Init the low level hardware */
		u = uart_hw_init(huart);
//	}

	huart->State = HAL_UART_STATE_BUSY;

	/* Disable the peripheral */
	__HAL_UART_DISABLE(huart);

	/* Set the UART Communication parameters */
	uart_setconfig(huart);

	/*
	 * In asynchronous mode, the following bits must be kept cleared:
	 * - LINEN and CLKEN bits in the USART_CR2 register,
	 * - SCEN, HDSEL and IREN  bits in the USART_CR3 register.
	 */
	huart->Instance->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	huart->Instance->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

	/* Enable the peripheral */
	__HAL_UART_ENABLE(huart);

	/* Initialize the UART state */
	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->State= HAL_UART_STATE_READY;

	return u;
}


int uart_setbaudrate(struct uart *u, int new_baudrate)
{
	static int last_baudrate = 4000000;
	int target_baudrate;

	/* baudrate of 0 means "next one" */
	if (new_baudrate == 0) {
		switch (last_baudrate) {
		case 115200:	target_baudrate = 460800; break;
		case 460800:	target_baudrate = 921600; break;
		case 921600:	target_baudrate = 2900000; break;
		case 2900000:	target_baudrate = 3200000; break;
		case 3200000:	target_baudrate = 3686400; break;
		case 3686400:	target_baudrate = 4000000; break;

		default:
			/* intentional fall-through */
		case 4000000:	target_baudrate = 115200; break;
		};

	} else {
		target_baudrate = new_baudrate;
	}

	printf("Setting up for %d\n", target_baudrate);

	/* use a case statement to be able to pick out and do funny things with baudrates, as needed */
	switch (target_baudrate) {
	default:
		target_baudrate = 115200;

		/* intentional fall-through */

	case 115200:
	case 460800:
	case 921600:
	case 2900000:
	case 3200000:
	case 3686400:
	case 4000000:
		u->huart->Init.BaudRate = target_baudrate;
		u->huart->Init.WordLength = UART_WORDLENGTH_8B;
		u->huart->Init.StopBits = UART_STOPBITS_1;
		u->huart->Init.Parity = UART_PARITY_NONE;
		u->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		u->huart->Init.Mode = UART_MODE_TX_RX;
		u->huart->Init.OverSampling = UART_OVERSAMPLING_16;
		break;
	};

	if (uart_init(u->huart) == NULL) {
		printf("unable to set baudrate!\n");
		Error_Handler();
	}

	last_baudrate = target_baudrate;
	return target_baudrate;
}



/* writes a single character, blocking until the character is written */
static int uart_write_nofifo(struct uart *u, const char ch)
{
	while (! (u->huart->Instance->SR & UART_FLAG_TXE)) ;
	u->huart->Instance->DR = ch;

	return 1;
}

/* writes a single character, from the UART if available */
static int uart_read_nofifo(struct uart *u, char *ch)
{
	if (u->huart->Instance->SR & UART_FLAG_RXNE) {
		*ch = u->huart->Instance->DR;
		return 1;
	} else {
		return 0;
	}
}


/* writes to the transmitter FIFO. Returns the number of bytes written */
int uart_write(struct uart *u, const char *buf, int len)
{
	int nwritten;

	nwritten = 0;
	while (len) {
		if (fifo_put(&u->tx, *buf)) {
			++buf;
			++nwritten;
			--len;
		} else {
			break;
		}

		/* enable the transmitter empty interrupt to start things flowing */
		__HAL_UART_ENABLE_IT(u->huart, UART_IT_TXE);
	};

	return nwritten;
}


/* reads from the receiver FIFO. Returns the number of bytes read */
int uart_read(struct uart *u, char *buf, int maxlen)
{
	int nread, left;

	nread = 0;
	left = maxlen;
	while (left) {
		if (fifo_get(&u->rx, buf)) {
			++buf;
			++nread;
			--left;

		/* no character available, drop out immediately */
		} else {
			break;
		}
	};

	return nread;
}


/* reads up to maxlen bytes or a newline (\n) character, for up to timeout msec. timeout of zero means don't wait. */
int uart_readline(struct uart *u, char *buf, int maxlen, int timeout)
{
	int nread, left;
	unsigned int maxtime;
	char *p_ch, *p_lastch;

	/* calculate exit time */
	maxtime = osKernelSysTick() + timeout;

	nread = 0;
	left = maxlen;
	p_lastch = buf;
	while (left && osKernelSysTick() <= maxtime) {
		if (fifo_get(&u->rx, buf)) {
			p_ch = buf;

			++buf;
			++nread;
			--left;

			/* trim off the \n and possibly \r\n */
			if (*p_ch == '\n') {
				*p_ch = 0;
				if (nread > 1 && *p_lastch == '\r') {
					--nread;
					*p_lastch = 0;
				}

				/* we're done */
				break;
			}

		/* no character available, give up our timeslice */
		} else {
			osThreadYield();
		}
	};

	return nread;
}


void uart_drain(struct uart *u)
{
	char c;

	while (uart_read(u, &c, 1)) ;
}


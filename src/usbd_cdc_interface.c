#include <cmsis_os.h>
#include <stdbool.h>
#include <usbd_cdc.h>
#include "main.h"
#include "critical.h"

#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

/* How often to check for outgoing data (to the Host), in msec. The max is 65 and the min is 1 */
#define CDC_POLLING_INTERVAL		(5)

USBD_CDC_LineCodingTypeDef LineCoding = {
	115200,		/* baud rate*/
	0x00,		/* stop bits-1*/
	0x00,		/* parity - none*/
	0x08		/* nb. of bits 8*/
};

static uint8_t UserRxBuffer[APP_RX_DATA_SIZE];			/* data received from the USB Host */
static volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];		/* data transmitted to the USB Host */
static uint32_t UserTxBufPtrIn;					/* head pointer (end of what we'll write to the Host) */
static uint32_t UserTxBufPtrOut;				/* tail pointer (start of what will be written to Host) */

TIM_HandleTypeDef  hcdc_timer;
extern USBD_HandleTypeDef USBD_Device;

static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);


static void cdc_timer_init(void)
{
	hcdc_timer.Instance = TIM4;

	__HAL_RCC_TIM4_CLK_ENABLE();

	/*
	 * Initialize TIM4 peripheral as follows:
	 * Period = 10000 - 1
	 * Prescaler = ((SystemCoreClock/2)/10000) - 1
	 * ClockDivision = 0
	  *Counter direction = Up
	 */
	hcdc_timer.Init.Period = (CDC_POLLING_INTERVAL * 1000) - 1;
	hcdc_timer.Init.Prescaler = 84 - 1;
	hcdc_timer.Init.ClockDivision = 0;
	hcdc_timer.Init.CounterMode = TIM_COUNTERMODE_UP;

	if (HAL_TIM_Base_Init(&hcdc_timer) != HAL_OK) {
		Error_Handler();
	}

	HAL_NVIC_SetPriority(TIM4_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}


/*
 * initializes what the CDC system talks to (e.g. a UART or whatever)
 * also sets up the periodic CDC callback timer to send whatever is in the
 * tx buffer to the host)
 */
static int8_t CDC_Itf_Init(void)
{
	cdc_timer_init();
	if (HAL_TIM_Base_Start_IT(&hcdc_timer) != HAL_OK) {
		Error_Handler();
	}

	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)UserTxBuffer, 0);
	USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);

	return USBD_OK;
}


static int8_t CDC_Itf_DeInit(void)
{
	return USBD_OK;
}


/* Manages the CDC class requests */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	switch (cmd) {
	case CDC_SEND_ENCAPSULATED_COMMAND:
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		break;

	case CDC_SET_COMM_FEATURE:
		break;

	case CDC_GET_COMM_FEATURE:
		break;

	case CDC_CLEAR_COMM_FEATURE:
		break;

	case CDC_SET_LINE_CODING:
		LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
		LineCoding.format     = pbuf[4];
		LineCoding.paritytype = pbuf[5];
		LineCoding.datatype   = pbuf[6];
		break;

	case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t)(LineCoding.bitrate);
		pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
		pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
		pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
		pbuf[4] = LineCoding.format;
		pbuf[5] = LineCoding.paritytype;
		pbuf[6] = LineCoding.datatype;
		break;

	case CDC_SET_CONTROL_LINE_STATE:
		break;

	case CDC_SEND_BREAK:
		break;

	default:
		break;
	};

	return USBD_OK;
}


/* callback for when the host sends data to us */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
	return USBD_OK;
}


USBD_CDC_ItfTypeDef USBD_CDC_fops = {
	CDC_Itf_Init,
	CDC_Itf_DeInit,
	CDC_Itf_Control,
	CDC_Itf_Receive
};


/* Timer callback. Send any outgoing data to the USB Host */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	DEFINE_CRITICAL();
	uint32_t buffptr;
	uint32_t buffsize;

	ENTER_CRITICAL();
	if (UserTxBufPtrOut != UserTxBufPtrIn) {
		if (UserTxBufPtrOut > UserTxBufPtrIn) {
			buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
		} else {
			buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
		}

		buffptr = UserTxBufPtrOut;

		USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)&UserTxBuffer[buffptr], buffsize);

		if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK) {
			UserTxBufPtrOut += buffsize;
			if (UserTxBufPtrOut >= APP_TX_DATA_SIZE) {
				UserTxBufPtrOut = 0;
			}
		}
	}
	EXIT_CRITICAL();
}


/* writes a character into the outgoing buffer, wrapping as necessary */
static bool cdc_fifo_put(const char val)
{
	int next;
	bool ret;

	/* calculate next head position to determine if the value will fit */
	next = UserTxBufPtrIn;
	++next;
	if (next >= APP_TX_DATA_SIZE) {
		next = 0;
	}

	/* if the FIFO isn't full (next == tail), write the value to the FIFO and update head. */
	if (next != UserTxBufPtrOut) {
		*(uint8_t *)&UserTxBuffer[UserTxBufPtrIn] = val;
		UserTxBufPtrIn = next;
		ret = true;
	} else {
		ret = false;
	}

	return ret;
}


/* writes to the transmitter FIFO. Returns the number of bytes written */
int cdc_write(const char *buf, int len)
{
	DEFINE_CRITICAL();
	int nwritten;

	ENTER_CRITICAL();
	nwritten = 0;
	while (len) {
		if (cdc_fifo_put(*buf)) {
			++buf;
			++nwritten;
			--len;
		} else {
			break;
		}
	};
	EXIT_CRITICAL();

	return nwritten;
}


int __io_putchar(int ch)
{
	if (ch == '\n') {
		char c = '\r';
		cdc_write(&c, 1);
	}

	return cdc_write((char *)&ch, 1);
}


int __io_getchar(void)
{
	return 0;
}


int fputc(int ch, FILE *fp)
{
	return __io_putchar(ch);
}

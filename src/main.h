#ifndef __MAIN_H
#define __MAIN_H

//#include "stm32f4xx_hal.h"

#define FIFO_LENGTH	(512)

#define MS_ONE_SECOND	(1000)

/*
 * head points to the location for the next value to be written.
 * tail points to the location for the next value to be read.
 * if head == tail, the FIFO is empty
 */
struct fifo {
	volatile int lock;
	volatile char *head;			/* where to write the next byte */
	volatile char *tail;			/* the last byte read */
	volatile char data[FIFO_LENGTH];
};

void dump(uint8_t *buf, int len);

void usb_init(void);

bool fifo_get(struct fifo *fifo, char *dest);
bool fifo_put(struct fifo *fifo, const char val);

void Error_Handler(void);

#endif /* __MAIN_H */

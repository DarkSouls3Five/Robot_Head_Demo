#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include "struct_typedef.h"

extern uint8_t sbus_buf[2][18];

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

#endif

#ifndef RS485_H
#define RS485_H

#include <stdarg.h>
#include "main.h"
#include "stm32f7xx_hal.h"

class RS485
{
public:
  enum {
    INPUT = 0,
    OUTPUT
  };

public:
  RS485(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx);

  void setDirection(int status);

  char putc(int c);
  
  int getc();
  
  int read(char *buf);
  
  int write(const void* buffer, size_t length);
  
  int printf(const char* format, ...);

private:
  UART_HandleTypeDef *_huart;
  DMA_HandleTypeDef *_hdma_usart_rx;
  DMA_HandleTypeDef *_hdma_usart_tx;

  static const int TX_BUF_SIZE = 16;
  static const int RX_BUF_SIZE = 256;
  unsigned char _tx_buf[TX_BUF_SIZE];
  unsigned char _rx_buf[RX_BUF_SIZE];
};

#endif

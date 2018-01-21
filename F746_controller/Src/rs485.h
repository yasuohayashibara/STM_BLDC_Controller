#ifndef RS485_H
#define RS485_H

#include <stdarg.h>
#include "main.h"
#include "stm32f7xx_hal.h"

class RS485
{
public:
  RS485(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx) :
    _huart(huart), _hdma_usart_rx(hdma_usart_rx), _hdma_usart_tx(hdma_usart_tx)
  {
    HAL_GPIO_WritePin(RX485_REDE_GPIO_Port, RX485_REDE_Pin, GPIO_PIN_SET);
  }
  
  char putc(int c)
  {
    _tx_buf[0] = c;
    return HAL_UART_Transmit_DMA(_huart, _tx_buf, 1) == HAL_OK ? c : EOF;
  }
  
  int getc()
  {
    return HAL_UART_Receive_DMA(_huart, _rx_buf, 1) == HAL_OK ? _rx_buf[0] : EOF;
  }
  
  int write(const void* buffer, size_t length)
  {
    unsigned char *buf = (unsigned char *)buffer;
    unsigned char *tx_buf = _tx_buf;
    for(int i = 0; i < length; i ++){
      *tx_buf ++ = *buf ++;
    }
    return HAL_UART_Transmit_DMA(_huart, _tx_buf, length) == HAL_OK ? length : 0;
  }
  
  int printf(const char* format, ...)
  {
    va_list arg;
    va_start(arg, format);
    char buf[256];
    int res = vsprintf(buf, format, arg);
    va_end(arg);
    write(buf, res);
    return res;
  }

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

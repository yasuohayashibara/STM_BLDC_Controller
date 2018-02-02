#include "RS485.h"

RS485 *p_rs485;
unsigned char temp_int[256];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    p_rs485->setDirection(RS485::INPUT);
    p_rs485->read(temp_int, 256);
  }
}

RS485::RS485(UART_HandleTypeDef *huart) :
  _huart(huart)
{
  setDirection(INPUT);
  p_rs485 = this;
}
  
void RS485::setDirection(int status) {
  GPIO_PinState pin_state = status == OUTPUT ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(RX485_REDE_GPIO_Port, RX485_REDE_Pin, pin_state);
}
  
char RS485::putc(int c)
{
  _tx_buf[0] = c;
  setDirection(OUTPUT);
  HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(_huart, _tx_buf, 1);
  return res == HAL_OK ? c : EOF;
}
  
int RS485::getc()
{
  return HAL_UART_Receive_DMA(_huart, _rx_buf, 1) == HAL_OK ? _rx_buf[0] : EOF;
}
  
int RS485::read(unsigned char *buf, unsigned int len)
{
  if (HAL_UART_Receive_DMA(_huart, _rx_buf, 7) != HAL_OK) return 0;
  
  int ret = 0;
  for(; ret < len; ret ++) {
    buf[ret] = _rx_buf[ret];
  }
  return ret;
}
  
int RS485::write(const void* buffer, size_t length)
{
  unsigned char *buf = (unsigned char *)buffer;
  unsigned char *tx_buf = _tx_buf;
  for(int i = 0; i < length; i ++){
    *tx_buf ++ = *buf ++;
  }
  setDirection(OUTPUT);
  while(HAL_UART_Transmit_DMA(_huart, _tx_buf, length) != HAL_OK);
  return length;
}
  
int RS485::printf(const char* format, ...)
{
  va_list arg;
  va_start(arg, format);
  char buf[256];
  int res = vsprintf(buf, format, arg);
  va_end(arg);
  write(buf, res);
  return res;
}

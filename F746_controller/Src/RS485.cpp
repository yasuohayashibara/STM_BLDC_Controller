#include "RS485.h"

RS485 *p_rs485;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    p_rs485->setDirection(RS485::INPUT);
    char temp[256];
    p_rs485->read(temp);
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
//  if (res != HAL_OK) setDirection(OUTPUT);
  return res == HAL_OK ? c : EOF;
}
  
int RS485::getc()
{
  return HAL_UART_Receive_DMA(_huart, _rx_buf, 1) == HAL_OK ? _rx_buf[0] : EOF;
}
  
int RS485::read(char *buf)
{
  int ret = 0;
  for(int c = getc(); c != EOF; ret ++) {
    c = getc();
    *buf ++ = c;
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
  HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(_huart, _tx_buf, length);
//  if (res == HAL_OK) setDirection(OUTPUT);
  return res == HAL_OK ? length : 0;
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

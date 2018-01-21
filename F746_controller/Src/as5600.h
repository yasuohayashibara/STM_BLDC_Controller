#ifndef AS5600_H
#define AS5600_H

#include "main.h"
#include "stm32f7xx_hal.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

class AS5600
{
public:
  AS5600(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma_i2c_rx, DMA_HandleTypeDef *hdma_i2c_tx) :
    _hi2c(hi2c), _hdma_i2c_rx(hdma_i2c_rx), _hdma_i2c_tx(hdma_i2c_tx)
  {
  }
  
  bool measureAngle()
  {
    _tx_buf[0] = ANGLE_ADRESS;
//    return HAL_I2C_Master_Transmit_DMA(_hi2c, SLAVE_ADRESS << 1, _tx_buf, 1) == HAL_OK ? true : false;
    return HAL_I2C_Master_Transmit(_hi2c, SLAVE_ADRESS << 1, _tx_buf, 1, 1000) == HAL_OK ? true : false;
  }
  
  bool getAngle(float *angle)
  {
//    if (HAL_I2C_Master_Receive_DMA(_hi2c, SLAVE_ADRESS << 1, _rx_buf, 2) == HAL_OK) {
    if (HAL_I2C_Master_Receive(_hi2c, SLAVE_ADRESS << 1, _rx_buf, 2, 1000) == HAL_OK) {
      *angle = ((_rx_buf[0] << 8) + _rx_buf[1]) * 0.087912087f * M_PI / 180.0f;
      return true;
    }
    return false;
  }
  
private:
  I2C_HandleTypeDef *_hi2c;
  DMA_HandleTypeDef *_hdma_i2c_rx;
  DMA_HandleTypeDef *_hdma_i2c_tx;

  static const int TX_BUF_SIZE = 16;
  static const int RX_BUF_SIZE = 16;
  unsigned char _tx_buf[TX_BUF_SIZE];
  unsigned char _rx_buf[RX_BUF_SIZE];

  static const int SLAVE_ADRESS = 0x36;
  static const int ANGLE_ADRESS = 0x0E;
};

#endif

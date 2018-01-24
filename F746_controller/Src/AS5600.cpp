#include "AS5600.h"

AS5600 *p_AS5600;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *huart)
{
  if (huart->Instance == I2C2)
  {
    if (p_AS5600->_do_measure)
      if (!p_AS5600->receiveAngleRequest()) p_AS5600->_error = true;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *huart)
{
  if (huart->Instance == I2C2)
  {
    p_AS5600->receiveAngle();
    if (!p_AS5600->sendMeasureAngleRequest()) p_AS5600->_error = true;
  }
}

AS5600::AS5600(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma_i2c_rx, DMA_HandleTypeDef *hdma_i2c_tx) :
  _do_measure(false), _error(false), _angle(0), _angle0(0),
  _hi2c(hi2c), _hdma_i2c_rx(hdma_i2c_rx), _hdma_i2c_tx(hdma_i2c_tx)
{
  p_AS5600 = this;
}

void AS5600::startMeasure()
{
  _do_measure = true;
  sendMeasureAngleRequest();
}

void AS5600::stopMeasure()
{
  _do_measure = false;
}

bool AS5600::sendMeasureAngleRequest()
{
  _tx_buf[0] = ANGLE_ADRESS;
  return HAL_I2C_Master_Transmit_DMA(_hi2c, SLAVE_ADRESS << 1, _tx_buf, 1) == HAL_OK ? true : false;
}

bool AS5600::receiveAngleRequest()
{
  return HAL_I2C_Master_Receive_DMA(_hi2c, SLAVE_ADRESS << 1, _rx_buf, 2) == HAL_OK ? true : false;
}

void AS5600::receiveAngle()
{
  _angle = maxPI(((_rx_buf[0] << 8) + _rx_buf[1]) * 0.087912087f * M_PI / 180.0f - _angle0);
}

float AS5600::maxPI(float angle)
{
  while(angle >  M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

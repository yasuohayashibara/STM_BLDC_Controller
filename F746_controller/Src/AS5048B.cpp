#include "AS5048B.h"

AS5048B *p_AS5048B;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *huart)
{
  if (huart->Instance == I2C2)
  {
    if (p_AS5048B->_do_measure)
      if (!p_AS5048B->receiveAngleRequest()) p_AS5048B->_error = true;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *huart)
{
  if (huart->Instance == I2C2)
  {
    p_AS5048B->receiveAngle();
    if (!p_AS5048B->sendMeasureAngleRequest()) p_AS5048B->_error = true;
  }
}

AS5048B::AS5048B(I2C_HandleTypeDef *hi2c) :
  _do_measure(false), _error(false), _angle(0), _angle0(0), _hi2c(hi2c)
{
  p_AS5048B = this;
}

void AS5048B::startMeasure()
{
  _do_measure = true;
  sendMeasureAngleRequest();
}

void AS5048B::stopMeasure()
{
  _do_measure = false;
}

bool AS5048B::sendMeasureAngleRequest()
{
  _tx_buf[0] = ANGLE_ADRESS;
  return HAL_I2C_Master_Transmit_DMA(_hi2c, SLAVE_ADRESS << 1, _tx_buf, 1) == HAL_OK ? true : false;
}

bool AS5048B::receiveAngleRequest()
{
  return HAL_I2C_Master_Receive_DMA(_hi2c, SLAVE_ADRESS << 1, _rx_buf, 2) == HAL_OK ? true : false;
}

void AS5048B::receiveAngle()
{
  _angle = maxPI((((unsigned short)_rx_buf[0] << 6) + (_rx_buf[1] & 0x3f)) * 0.021973997f * M_PI / 180.0f - _angle0);
  _angle = -_angle;
}

float AS5048B::maxPI(float angle)
{
  while(angle >  M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

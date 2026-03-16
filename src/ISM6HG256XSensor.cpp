/**
 ******************************************************************************
 * @file    ISM6HG256XSensor.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    05 August 2025
 * @brief   Implementation of a ISM6HG256X sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ISM6HG256XSensor.h"
/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the ism6hg256x's instance
 */
ISM6HG256XSensor::ISM6HG256XSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = ISM6HG256X_io_write;
  reg_ctx.read_reg = ISM6HG256X_io_read;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  is_initialized = 0;
  acc_is_enabled = 0;
  acc_hg_is_enabled = 0;
  gyro_is_enabled = 0;
  acc_mode = ISM6HG256X_ACC_HIGH_PERFORMANCE_MODE;
  gyro_mode = ISM6HG256X_GYRO_HIGH_PERFORMANCE_MODE;
}
/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
ISM6HG256XSensor::ISM6HG256XSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = ISM6HG256X_io_write;
  reg_ctx.read_reg = ISM6HG256X_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  is_initialized = 0;
  acc_is_enabled = 0;
  acc_hg_is_enabled = 0;
  gyro_is_enabled = 0;
  acc_mode = ISM6HG256X_ACC_HIGH_PERFORMANCE_MODE;
  gyro_mode = ISM6HG256X_GYRO_HIGH_PERFORMANCE_MODE;
}
/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }
  /* Set main memory bank */
  if (Set_Mem_Bank((uint8_t)ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (ism6hg256x_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable BDU */
  if (ism6hg256x_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* WAKE_UP mode selection */
  if (ism6hg256x_fifo_mode_set(&reg_ctx, ISM6HG256X_BYPASS_MODE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Select default output data rate. */
  acc_odr = 120.0f;
  /* Output data rate selection - power down. */
  if (ism6hg256x_xl_data_rate_set(&reg_ctx, ISM6HG256X_ODR_OFF) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection. */
  if (ism6hg256x_xl_full_scale_set(&reg_ctx, ISM6HG256X_4g) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Select default output data rate. */
  acc_hg_odr = 480.0f;
  /* Output data rate selection - power down. */
  if (ism6hg256x_hg_xl_data_rate_set(&reg_ctx, ISM6HG256X_HG_XL_ODR_OFF, PROPERTY_ENABLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection. */
  if (ism6hg256x_hg_xl_full_scale_set(&reg_ctx, ISM6HG256X_32g) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Select default output data rate. */
  gyro_odr = 120.0f;
  /* Output data rate selection - power down. */
  if (ism6hg256x_gy_data_rate_set(&reg_ctx, ISM6HG256X_ODR_OFF) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection. */
  if (ism6hg256x_gy_full_scale_set(&reg_ctx, ISM6HG256X_2000dps) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  is_initialized = 1;
  return ISM6HG256X_OK;
}
/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::end()
{
  if (is_initialized == 1U && Disable_X() != ISM6HG256X_OK && Disable_G() != ISM6HG256X_OK && Disable_HG_X() != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }
  /* Disable the component */
  if (Disable_X() != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (Disable_HG_X() != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (Disable_G() != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset output data rate. */
  acc_odr = ISM6HG256X_ODR_OFF;
  acc_hg_odr = ISM6HG256X_HG_XL_ODR_OFF;
  gyro_odr = ISM6HG256X_ODR_OFF;
  is_initialized = 0;
  return ISM6HG256X_OK;
}
/**
  * @brief  Read component ID
  * @param  Id the WHO_AM_I value
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::ReadID(uint8_t *Id)
{
  if (ism6hg256x_device_id_get(&reg_ctx, Id) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable high-accuracy ODR modes for both the accelerometer and gyroscope
* @pre    The accelerometer and gyroscope must both be in power-down mode.
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_High_Accuracy_Mode(void)
{
  uint8_t ctrl_registers[2];
  ism6hg256x_ctrl1_t *ctrl1;
  ism6hg256x_ctrl2_t *ctrl2;

  /* error if either sensor has been enabled (powered on) already */
  if (acc_is_enabled || gyro_is_enabled) {
    return ISM6HG256X_ERROR;
  }

  /* read control registers */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_CTRL1, ctrl_registers, 2) != 0) {
    return ISM6HG256X_ERROR;
  }

  ctrl1 = (ism6hg256x_ctrl1_t *)&ctrl_registers[0];
  ctrl2 = (ism6hg256x_ctrl2_t *)&ctrl_registers[1];

  /* set high-accuracy modes for accelerometer and gyroscope */
  ctrl1->op_mode_xl = ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD;
  ctrl2->op_mode_g = ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD;

  /* write back to registers */
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_CTRL1, ctrl_registers, 2) != 0) {
    return ISM6HG256X_ERROR;
  }

  acc_mode = ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE;
  gyro_mode = ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE;

  return ISM6HG256X_OK;
}
/**
* @brief  Set the high-accuracy ODR range selection
* @pre    The gyroscope and accelerometer must be in high-accuracy mode
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_High_Accuracy_ODR_Range(ism6hg256x_haodr_sel_t sel)
{
  /* error if not both of the gyro and accelerometer are in high accuracy mode */
  if (acc_mode != ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE || gyro_mode != ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE) {
    return ISM6HG256X_ERROR;
  }

  /* write selection to register */
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&sel, 1) != 0) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}
/**
* @brief  Enable the ISM6HG256X accelerometer sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    return ISM6HG256X_OK;
  }

  /* Set the configured accelerometer ODR */
  if (Set_X_OutputDataRate(acc_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  acc_is_enabled = 1;
  return ISM6HG256X_OK;
}
/**
* @brief  Disable the ISM6HG256X accelerometer sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U) {
    return ISM6HG256X_OK;
  }
  /* Get current output data rate. */
  if (Get_X_OutputDataRate(&acc_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  /* Output data rate selection - power down. */
  if (ism6hg256x_xl_data_rate_set(&reg_ctx, ISM6HG256X_ODR_OFF) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  acc_is_enabled = 0;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X accelerometer sensor sensitivity
* @param  Sensitivity pointer
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_Sensitivity(float_t *Sensitivity)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_xl_full_scale_t full_scale;
  /* Read actual full scale selection from sensor. */
  if (ism6hg256x_xl_full_scale_get(&reg_ctx, &full_scale) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case ISM6HG256X_2g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_2G;
      break;
    case ISM6HG256X_4g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_4G;
      break;
    case ISM6HG256X_8g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_8G;
      break;
    case ISM6HG256X_16g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_16G;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Get the ISM6HG256X accelerometer sensor output data rate
* @param  Odr pointer where the output data rate is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_OutputDataRate(float_t *Odr)
{
  ism6hg256x_data_rate_t odr_low_level;
  /* Get current output data rate. */
  if (ism6hg256x_xl_data_rate_get(&reg_ctx, &odr_low_level) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  if (ism6hg256x_data_rate_to_float(odr_low_level, Odr) != 0) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X accelerometer sensor output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_OutputDataRate(float_t Odr)
{
  if (acc_is_enabled == 1U) {
    return Set_X_OutputDataRate_When_Enabled(Odr);
  } else {
    return Set_X_OutputDataRate_When_Disabled(Odr);
  }
}
/**
* @brief  Set the ISM6HG256X accelerometer sensor output data rate with operating mode
* @param  Odr the output data rate value to be set
* @param  Mode the accelerometer operating mode
* @note   This function switches off the gyroscope if Ultra Low Power Mode is set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_OutputDataRate_With_Mode(float_t Odr,
                                                                         ISM6HG256X_ACC_Operating_Mode_t Mode)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  float_t odr_new = 0.0f;
  switch (Mode) {
    case ISM6HG256X_ACC_HIGH_PERFORMANCE_MODE:
      if (ism6hg256x_xl_mode_set(&reg_ctx, ISM6HG256X_XL_HIGH_PERFORMANCE_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
      odr_new = (Odr <    7.5f) ?    7.5f
                : (Odr > 7680.0f) ? 7680.0f
                :                       Odr;
      break;
    case ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE:
      ret = ISM6HG256X_ERROR;
      break;
    case ISM6HG256X_ACC_ODR_TRIGGERED_MODE:
      ret = ISM6HG256X_ERROR;
      break;
    case ISM6HG256X_ACC_NORMAL_MODE:
      if (ism6hg256x_xl_mode_set(&reg_ctx, ISM6HG256X_XL_NORMAL_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 7.5Hz <= Odr <= 1.92kHz */
      odr_new = (Odr <    7.5f) ?    7.5f
                : (Odr > 1920.0f) ? 1920.0f
                :                       Odr;
      break;
    case ISM6HG256X_ACC_LOW_POWER_MODE1:
      if (ism6hg256x_xl_mode_set(&reg_ctx, ISM6HG256X_XL_LOW_POWER_2_AVG_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
      odr_new = (Odr ==   1.875f) ?    Odr
                : (Odr <   15.000f) ?  15.0f
                : (Odr >  240.000f) ? 240.0f
                :                        Odr;
      break;
    case ISM6HG256X_ACC_LOW_POWER_MODE2:
      if (ism6hg256x_xl_mode_set(&reg_ctx, ISM6HG256X_XL_LOW_POWER_4_AVG_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
      odr_new = (Odr ==   1.875f) ?    Odr
                : (Odr <   15.000f) ?  15.0f
                : (Odr >  240.000f) ? 240.0f
                :                        Odr;
      break;
    case ISM6HG256X_ACC_LOW_POWER_MODE3:
      if (ism6hg256x_xl_mode_set(&reg_ctx, ISM6HG256X_XL_LOW_POWER_8_AVG_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
      odr_new = (Odr ==   1.875f) ?    Odr
                : (Odr <   15.000f) ?  15.0f
                : (Odr >  240.000f) ? 240.0f
                :                        Odr;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  if (ret != ISM6HG256X_OK) {
    return ret;
  }
  if (acc_is_enabled == 1U) {
    ret = Set_X_OutputDataRate_When_Enabled(odr_new);
  } else {
    ret = Set_X_OutputDataRate_When_Disabled(odr_new);
  }
  return ret;
}
/**
* @brief  Get the ISM6HG256X accelerometer sensor full scale
* @param  FullScale pointer where the full scale is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_FullScale(int32_t *FullScale)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_xl_full_scale_t fs_low_level;
  /* Read actual full scale selection from sensor. */
  if (ism6hg256x_xl_full_scale_get(&reg_ctx, &fs_low_level) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  switch (fs_low_level) {
    case ISM6HG256X_2g:
      *FullScale =  2;
      break;
    case ISM6HG256X_4g:
      *FullScale =  4;
      break;
    case ISM6HG256X_8g:
      *FullScale =  8;
      break;
    case ISM6HG256X_16g:
      *FullScale = 16;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM6HG256X accelerometer sensor full scale
* @param  FullScale the functional full scale to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_FullScale(int32_t FullScale)
{
  ism6hg256x_xl_full_scale_t new_fs;
  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file static analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? ISM6HG256X_2g
           : (FullScale <= 4) ? ISM6HG256X_4g
           : (FullScale <= 8) ? ISM6HG256X_8g
           :                    ISM6HG256X_16g;
  if (ism6hg256x_xl_full_scale_set(&reg_ctx, new_fs) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X accelerometer sensor raw axes
* @param  Value pointer where the raw values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_AxesRaw(ISM6HG256X_AxesRaw_t *Value)
{
  ism6hg256x_axis3bit16_t data_raw;
  /* Read raw data values. */
  if (ism6hg256x_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Format the data. */
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X accelerometer sensor axes
* @param  Acceleration pointer where the values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_Axes(ISM6HG256X_Axes_t *Acceleration)
{
  ism6hg256x_axis3bit16_t data_raw;
  float_t sensitivity = 0.0f;
  /* Read raw data values. */
  if (ism6hg256x_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Get ISM6HG256X actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Calculate the data. */
  Acceleration->x = (int32_t)((float_t)((float_t)data_raw.i16bit[0] * sensitivity));
  Acceleration->y = (int32_t)((float_t)((float_t)data_raw.i16bit[1] * sensitivity));
  Acceleration->z = (int32_t)((float_t)((float_t)data_raw.i16bit[2] * sensitivity));
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of all hardware events
* @param  Status the status of all hardware events
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_Event_Status(ISM6HG256X_Event_Status_t *Status)
{
  ism6hg256x_emb_func_status_t emb_func_status;
  ism6hg256x_wake_up_src_t wake_up_src;
  ism6hg256x_tap_src_t tap_src;
  ism6hg256x_d6d_src_t d6d_src;
  ism6hg256x_emb_func_src_t func_src;
  ism6hg256x_md1_cfg_t md1_cfg;
  ism6hg256x_md2_cfg_t md2_cfg;
  ism6hg256x_emb_func_int1_t int1_ctrl;
  ism6hg256x_emb_func_int2_t int2_ctrl;
  (void)memset((void *)Status, 0x0, sizeof(ISM6HG256X_Event_Status_t));
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_SRC, (uint8_t *)&tap_src, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&d6d_src, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_SRC, (uint8_t *)&func_src, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&int1_ctrl, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&int2_ctrl, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_STATUS, (uint8_t *)&emb_func_status, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != 0) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&md1_cfg, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&md2_cfg, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U)) {
    if (wake_up_src.ff_ia == 1U) {
      Status->FreeFallStatus = 1;
    }
  }
  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U)) {
    if (wake_up_src.wu_ia == 1U) {
      Status->WakeUpStatus = 1;
    }
  }
  if ((md1_cfg.int1_single_tap == 1U) || (md2_cfg.int2_single_tap == 1U)) {
    if (tap_src.single_tap == 1U) {
      Status->TapStatus = 1;
    }
  }
  if ((md1_cfg.int1_double_tap == 1U) || (md2_cfg.int2_double_tap == 1U)) {
    if (tap_src.double_tap == 1U) {
      Status->DoubleTapStatus = 1;
    }
  }
  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U)) {
    if (d6d_src.d6d_ia == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }
  if ((int1_ctrl.int1_step_detector == 1U) || (int2_ctrl.int2_step_detector == 1U)) {
    if (func_src.step_detected == 1U) {
      Status->StepStatus = 1;
    }
  }
  if ((int1_ctrl.int1_tilt == 1U) || (int2_ctrl.int2_tilt == 1U)) {
    if (emb_func_status.is_tilt == 1U) {
      Status->TiltStatus = 1;
    }
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable free fall detection
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_Free_Fall_Detection(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_functions_enable_t functions_enable;
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(480.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(2) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /*  Set free fall duration.*/
  if (Set_Free_Fall_Duration(3) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set free fall threshold. */
  if (Set_Free_Fall_Threshold(3) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_ff = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_ff = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable free fall detection
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_Free_Fall_Detection()
{
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  /* Disable free fall event on both INT1 and INT2 pins */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_ff = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_ff = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset free fall threshold. */
  if (Set_Free_Fall_Threshold(0) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset free fall duration */
  if (Set_Free_Fall_Duration(0) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set free fall threshold
* @param  Threshold free fall detection threshold
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Free_Fall_Threshold(uint8_t Threshold)
{
  ism6hg256x_ff_thresholds_t val;
  switch (Threshold) {
    case ISM6HG256X_156_mg:
      val = ISM6HG256X_156_mg;
      break;
    case ISM6HG256X_219_mg:
      val = ISM6HG256X_219_mg;
      break;
    case ISM6HG256X_250_mg:
      val = ISM6HG256X_250_mg;
      break;
    case ISM6HG256X_312_mg:
      val = ISM6HG256X_312_mg;
      break;
    case ISM6HG256X_344_mg:
      val = ISM6HG256X_344_mg;
      break;
    case ISM6HG256X_406_mg:
      val = ISM6HG256X_406_mg;
      break;
    case ISM6HG256X_469_mg:
      val = ISM6HG256X_469_mg;
      break;
    case ISM6HG256X_500_mg:
      val = ISM6HG256X_500_mg;
      break;
    default:
      val = ISM6HG256X_156_mg;
      break;
  }
  /* Set free fall threshold. */
  if (ism6hg256x_ff_thresholds_set(&reg_ctx, val) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set free fall duration
* @param  Duration free fall detection duration
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Free_Fall_Duration(uint8_t Duration)
{
  /* Set free fall duration. */
  if (ism6hg256x_ff_time_windows_set(&reg_ctx, Duration) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable tilt detection
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_Tilt_Detection(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_emb_func_en_a_t emb_func_en_a;
  ism6hg256x_emb_func_int1_t emb_func_int1;
  ism6hg256x_emb_func_int2_t emb_func_int2;
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(30.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(2) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      /* Enable access to embedded functions registers */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Enable tilt detection */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      emb_func_en_a.tilt_en = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Tilt interrupt driven to INT1 pin */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      emb_func_int1.int1_tilt = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Disable access to embedded functions registers */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Enable routing the embedded functions interrupt */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_emb_func = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      /* Enable access to embedded functions registers */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Enable tilt detection */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      emb_func_en_a.tilt_en = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Tilt interrupt driven to INT2 pin */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      emb_func_int2.int2_tilt = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Disable access to embedded functions registers */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Enable routing the embedded functions interrupt */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_emb_func = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable tilt detection
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_Tilt_Detection()
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_emb_func_en_a_t emb_func_en_a;
  ism6hg256x_emb_func_int1_t emb_func_int1;
  ism6hg256x_emb_func_int2_t emb_func_int2;
  /* Disable emb func event on either INT1 or INT2 pin */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_emb_func = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_emb_func = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable access to embedded functions registers. */
  if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable tilt detection. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  emb_func_en_a.tilt_en = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset interrupt driven to INT1 pin. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  emb_func_int1.int1_tilt = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset interrupt driven to INT2 pin. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  emb_func_int2.int2_tilt = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable access to embedded functions registers. */
  if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ret;
}
/**
* @brief  Enable pedometer
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_Pedometer(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_stpcnt_mode_t mode;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_emb_func_int1_t emb_func_int1;
  ism6hg256x_emb_func_int2_t emb_func_int2;
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(30.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(8) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_stpcnt_mode_get(&reg_ctx, &mode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_ENABLE;
  mode.false_step_rej = PROPERTY_DISABLE;
  /* Turn on embedded features */
  if (ism6hg256x_stpcnt_mode_set(&reg_ctx, mode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      /* Enable access to embedded functions registers. */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Step detector interrupt driven to INT1 pin */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Disable access to embedded functions registers */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_emb_func = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      /* Enable access to embedded functions registers. */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Step detector interrupt driven to INT1 pin */
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      emb_func_int2.int2_step_detector = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Disable access to embedded functions registers */
      if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_emb_func = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable pedometer
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_Pedometer()
{
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_emb_func_int1_t emb_func_int1;
  ism6hg256x_emb_func_int2_t emb_func_int2;
  ism6hg256x_stpcnt_mode_t mode;
  if (ism6hg256x_stpcnt_mode_get(&reg_ctx, &mode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_DISABLE;
  mode.false_step_rej = PROPERTY_DISABLE;
  /* Turn off embedded features */
  if (ism6hg256x_stpcnt_mode_set(&reg_ctx, mode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable emb func event on either INT1 or INT2 pin */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_emb_func = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_emb_func = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable access to embedded functions registers. */
  if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset interrupt driven to INT1 pin. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  emb_func_int1.int1_step_detector = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset interrupt driven to INT2 pin. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  emb_func_int2.int2_step_detector = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable access to embedded functions registers. */
  if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get step count
* @param  StepCount step counter
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_Step_Count(uint16_t *StepCount)
{
  if (ism6hg256x_stpcnt_steps_get(&reg_ctx, StepCount) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable step counter reset
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Step_Counter_Reset()
{
  if (ism6hg256x_stpcnt_rst_step_set(&reg_ctx, PROPERTY_ENABLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable wake up detection
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_Wake_Up_Detection(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_functions_enable_t functions_enable;
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(480.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(2) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set wake-up threshold */
  if (Set_Wake_Up_Threshold(1, 3) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set wake-up durantion */
  if (Set_Wake_Up_Duration(0) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_wu = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_wu = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable wake up detection
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_Wake_Up_Detection()
{
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  /* Disable wake up event on both INT1 and INT2 pins */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_wu = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_wu = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset wake-up threshold */
  if (Set_Wake_Up_Threshold(0, 0) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset wake-up durantion */
  if (Set_Wake_Up_Duration(0) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set wake up threshold
* @param  Threshold wake up detection threshold
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Wake_Up_Threshold(uint32_t Threshold, uint32_t InactivityThreshold)
{
  ism6hg256x_act_thresholds_t wake_up_ths;
  if (ism6hg256x_act_thresholds_get(&reg_ctx, &wake_up_ths) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  wake_up_ths.inactivity_cfg.wu_inact_ths_w = (uint8_t)InactivityThreshold;
  wake_up_ths.threshold = (uint8_t)Threshold;
  if (ism6hg256x_act_thresholds_set(&reg_ctx, &wake_up_ths) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set wake up duration
* @param  Duration wake up detection duration
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Wake_Up_Duration(uint8_t Duration)
{
  ism6hg256x_act_thresholds_t wake_up_ths;
  if (ism6hg256x_act_thresholds_get(&reg_ctx, &wake_up_ths) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  wake_up_ths.duration = (uint8_t)Duration;
  if (ism6hg256x_act_thresholds_set(&reg_ctx, &wake_up_ths) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable single tap detection
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_Single_Tap_Detection(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_functions_enable_t functions_enable;
  ism6hg256x_tap_dur_t tap_dur;
  ism6hg256x_tap_cfg0_t tap_cfg0;
  ism6hg256x_tap_ths_6d_t tap_ths_6d;
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(480.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(8) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable tap detection on Z-axis. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_cfg0.tap_z_en = 0x01U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set Z-axis threshold. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_ths_6d.tap_ths_z = 0x2U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set quiet and shock time windows. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.quiet = (uint8_t)0x02U;
  tap_dur.shock = (uint8_t)0x01U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set tap mode. */
  if (ism6hg256x_tap_mode_set(&reg_ctx, ISM6HG256X_ONLY_SINGLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_single_tap = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_single_tap = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable single tap detection
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_Single_Tap_Detection()
{
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_tap_dur_t tap_dur;
  ism6hg256x_tap_cfg0_t tap_cfg0;
  ism6hg256x_tap_ths_6d_t tap_ths_6d;
  /* Disable single tap event on both INT1 and INT2 pins */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_single_tap = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_single_tap = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable tap detection on Z-axis. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_cfg0.tap_z_en = 0x0U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset Z-axis threshold. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_ths_6d.tap_ths_z = 0x0U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset quiet and shock time windows. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.quiet = (uint8_t)0x0U;
  tap_dur.shock = (uint8_t)0x0U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable double tap detection
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_Double_Tap_Detection(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_functions_enable_t functions_enable;
  ism6hg256x_tap_dur_t tap_dur;
  ism6hg256x_tap_cfg0_t tap_cfg0;
  ism6hg256x_tap_ths_6d_t tap_ths_6d;
  /* Enable tap detection on Z-axis. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_cfg0.tap_z_en = 0x01U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set Z-axis threshold. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_ths_6d.tap_ths_z = 0x03U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set quiet shock and duration. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.dur = (uint8_t)0x03U;
  tap_dur.quiet = (uint8_t)0x02U;
  tap_dur.shock = (uint8_t)0x02U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set tap mode. */
  if (ism6hg256x_tap_mode_set(&reg_ctx, ISM6HG256X_BOTH_SINGLE_DOUBLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(480.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(8) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_double_tap = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_double_tap = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable double tap detection
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_Double_Tap_Detection()
{
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_tap_dur_t tap_dur;
  ism6hg256x_tap_cfg0_t tap_cfg0;
  ism6hg256x_tap_ths_6d_t tap_ths_6d;
  /* Disable double tap event on both INT1 and INT2 pins */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_ff = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_ff = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable tap detection on Z-axis. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_cfg0.tap_z_en = 0x0U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset Z-axis threshold. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_ths_6d.tap_ths_z = 0x0U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Reset quiet shock and duration. */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.dur = (uint8_t)0x0U;
  tap_dur.quiet = (uint8_t)0x0U;
  tap_dur.shock = (uint8_t)0x0U;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Set tap mode. */
  if (ism6hg256x_tap_mode_set(&reg_ctx, ISM6HG256X_ONLY_SINGLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set tap threshold
* @param  Threshold tap threshold
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Tap_Threshold(uint8_t Threshold)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d;
  /* Set Z-axis threshold */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_ths_6d.tap_ths_z = Threshold;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_ths_6d, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set tap duration time
* @param  Time shock duration time
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Tap_Shock_Time(uint8_t Time)
{
  ism6hg256x_tap_dur_t tap_dur;
  /* Set shock time */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.shock = Time;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set tap duration time
* @param  Time tap quiet time
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Tap_Quiet_Time(uint8_t Time)
{
  ism6hg256x_tap_dur_t tap_dur;
  /* Set quiet time */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.quiet = Time;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set tap duration time
* @param  Time tap duration time
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Tap_Duration_Time(uint8_t Time)
{
  ism6hg256x_tap_dur_t tap_dur;
  /* Set duration time */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  tap_dur.dur = Time;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable 6d orientation
* @param  IntPin interrupt pin line to be used
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_6D_Orientation(ISM6HG256X_SensorIntPin_t IntPin)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  ism6hg256x_functions_enable_t functions_enable;
  /* Output Data Rate selection */
  if (Set_X_OutputDataRate(480.0f) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FullScale(2) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Threshold selection*/
  if (Set_6D_Orientation_Threshold(2) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin) {
    case ISM6HG256X_INT1_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val1.int1_6d = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    case ISM6HG256X_INT2_PIN:
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      val2.int2_6d = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      functions_enable.interrupts_enable = PROPERTY_ENABLE;
      if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Disable 6D orientation detection
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_6D_Orientation()
{
  ism6hg256x_md1_cfg_t val1;
  ism6hg256x_md2_cfg_t val2;
  /* Reset threshold */
  if (Set_6D_Orientation_Threshold(0) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val1.int1_6d = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&val1, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  val2.int2_6d = PROPERTY_DISABLE;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&val2, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set 6D orientation threshold
* @param  Threshold free fall detection threshold
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_6D_Orientation_Threshold(uint8_t Threshold)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_6d_threshold_t newThreshold = ISM6HG256X_DEG_80;
  switch (Threshold) {
    case 0:
      newThreshold = ISM6HG256X_DEG_80;
      break;
    case 1:
      newThreshold = ISM6HG256X_DEG_70;
      break;
    case 2:
      newThreshold = ISM6HG256X_DEG_60;
      break;
    case 3:
      newThreshold = ISM6HG256X_DEG_50;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  if (ret == ISM6HG256X_ERROR) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_6d_threshold_set(&reg_ctx, newThreshold) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of XLow orientation
* @param  XLow the status of XLow orientation
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_6D_Orientation_XL(uint8_t *XLow)
{
  ism6hg256x_d6d_src_t data;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *XLow = data.xl;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of XHigh orientation
* @param  XHigh the status of XHigh orientation
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_6D_Orientation_XH(uint8_t *XHigh)
{
  ism6hg256x_d6d_src_t data;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *XHigh = data.xh;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of YLow orientation
* @param  YLow the status of YLow orientation
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_6D_Orientation_YL(uint8_t *YLow)
{
  ism6hg256x_d6d_src_t data;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *YLow = data.yl;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of YHigh orientation
* @param  YHigh the status of YHigh orientation
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_6D_Orientation_YH(uint8_t *YHigh)
{
  ism6hg256x_d6d_src_t data;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *YHigh = data.yh;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of ZLow orientation
* @param  ZLow the status of ZLow orientation
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_6D_Orientation_ZL(uint8_t *ZLow)
{
  ism6hg256x_d6d_src_t data;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *ZLow = data.zl;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the status of ZHigh orientation
* @param  ZHigh the status of ZHigh orientation
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_6D_Orientation_ZH(uint8_t *ZHigh)
{
  ism6hg256x_d6d_src_t data;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_D6D_SRC, (uint8_t *)&data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *ZHigh = data.zh;
  return ISM6HG256X_OK;
}
/**
* @brief  Enable the ISM6HG256X high-g accelerometer sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_HG_X()
{
  /* Check if the component is already enabled */
  if (acc_hg_is_enabled == 1U) {
    return ISM6HG256X_OK;
  }
  /* Output data rate selection. */
  if (Set_HG_X_OutputDataRate(acc_hg_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  acc_hg_is_enabled = 1;
  return ISM6HG256X_OK;
}
/**
* @brief  Disable the ISM6HG256X high-g accelerometer sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_HG_X()
{
  uint8_t output_enable;
  /* Check if the component is already disabled */
  if (acc_hg_is_enabled == 0U) {
    return ISM6HG256X_OK;
  }
  /* Get current output data rate. */
  if (Get_HG_X_OutputDataRate(&acc_hg_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Output data rate selection - power down. */
  if (ism6hg256x_hg_xl_data_rate_set(&reg_ctx, ISM6HG256X_HG_XL_ODR_OFF, PROPERTY_ENABLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  acc_hg_is_enabled = 0;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X high-g accelerometer sensor sensitivity
* @param  Sensitivity pointer
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_HG_X_Sensitivity(float_t *Sensitivity)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_hg_xl_full_scale_t full_scale;
  /* Read actual full scale selection from sensor. */
  if (ism6hg256x_hg_xl_full_scale_get(&reg_ctx, &full_scale) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case ISM6HG256X_32g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_32G;
      break;
    case ISM6HG256X_64g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_64G;
      break;
    case ISM6HG256X_128g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_128G;
      break;
    case ISM6HG256X_256g:
      *Sensitivity = ISM6HG256X_ACC_SENSITIVITY_FS_256G;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Get the ISM6HG256X high-g accelerometer sensor output data rate
* @param  Odr pointer where the output data rate is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_HG_X_OutputDataRate(float_t *Odr)
{
  ism6hg256x_hg_xl_data_rate_t odr_low_level;
  uint8_t output_enable;
  /* Get current output data rate. */
  if (ism6hg256x_hg_xl_data_rate_get(&reg_ctx, &odr_low_level, &output_enable) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  if (ism6hg256x_hg_data_rate_to_float(odr_low_level, Odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X high-g accelerometer sensor output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_HG_X_OutputDataRate(float_t Odr)
{
  if (acc_is_enabled == 1U) {
    return Set_X_HG_OutputDataRate_When_Enabled(Odr);
  } else {
    return Set_X_HG_OutputDataRate_When_Disabled(Odr);
  }
}
/**
* @brief  Get the ISM6HG256X high-g accelerometer sensor full scale
* @param  FullScale pointer where the full scale is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::HG_X_GetFullScale(int32_t *FullScale)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_hg_xl_full_scale_t fs_low_level;
  /* Read actual full scale selection from sensor. */
  if (ism6hg256x_hg_xl_full_scale_get(&reg_ctx, &fs_low_level) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  switch (fs_low_level) {
    case ISM6HG256X_32g:
      *FullScale =  32;
      break;
    case ISM6HG256X_64g:
      *FullScale =  64;
      break;
    case ISM6HG256X_128g:
      *FullScale =  128;
      break;
    case ISM6HG256X_256g:
      *FullScale =  256;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM6HG256X high-g accelerometer sensor full scale
* @param  FullScale the functional full scale to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_HG_X_FullScale(int32_t FullScale)
{
  ism6hg256x_hg_xl_full_scale_t new_fs;
  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file static analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 32) ?  ISM6HG256X_32g
           : (FullScale <= 64) ?  ISM6HG256X_64g
           : (FullScale <= 128) ? ISM6HG256X_128g
           :                      ISM6HG256X_256g;
  if (ism6hg256x_hg_xl_full_scale_set(&reg_ctx, new_fs) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X high-g accelerometer sensor raw axes
* @param  Value pointer where the raw values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_HG_X_AxesRaw(ISM6HG256X_AxesRaw_t *Value)
{
  ism6hg256x_axis3bit16_t data_raw;
  /* Read raw data values. */
  if (ism6hg256x_hg_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Format the data. */
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X high-g accelerometer sensor axes
* @param  Acceleration pointer where the values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::HG_X_GetAxes(ISM6HG256X_Axes_t *Acceleration)
{
  ism6hg256x_axis3bit16_t data_raw;
  float_t sensitivity = 0.0f;
  /* Read raw data values. */
  if (ism6hg256x_hg_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Get ISM6HG256X actual sensitivity. */
  if (Get_HG_X_Sensitivity(&sensitivity) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Calculate the data. */
  Acceleration->x = (int32_t)((float_t)((float_t)data_raw.i16bit[0] * sensitivity));
  Acceleration->y = (int32_t)((float_t)((float_t)data_raw.i16bit[1] * sensitivity));
  Acceleration->z = (int32_t)((float_t)((float_t)data_raw.i16bit[2] * sensitivity));
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X FIFO number of samples
* @param  NumSamples number of samples
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Get_Num_Samples(uint16_t *NumSamples)
{
  ism6hg256x_fifo_status_t val;
  if (ism6hg256x_fifo_status_get(&reg_ctx, &val) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *NumSamples = val.fifo_level;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X FIFO full status
* @param  Status FIFO full status
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Get_Full_Status(uint8_t *Status)
{
  ism6hg256x_fifo_status_t val;
  if (ism6hg256x_fifo_status_get(&reg_ctx, &val) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *Status = val.fifo_full;
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO full interrupt on INT1 pin
* @param  Status FIFO full interrupt on INT1 pin status
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Set_INT1_FIFO_Full(uint8_t Status)
{
  ism6hg256x_int1_ctrl_t reg;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_INT1_CTRL, (uint8_t *)&reg, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  reg.int1_fifo_full = Status;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_INT1_CTRL, (uint8_t *)&reg, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO full interrupt on INT2 pin
* @param  Status FIFO full interrupt on INT1 pin status
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Set_INT2_FIFO_Full(uint8_t Status)
{
  ism6hg256x_int2_ctrl_t reg;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_INT2_CTRL, (uint8_t *)&reg, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  reg.int2_fifo_full = Status;
  if (ism6hg256x_write_reg(&reg_ctx, ISM6HG256X_INT2_CTRL, (uint8_t *)&reg, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO watermark level
* @param  Watermark FIFO watermark level
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Set_Watermark_Level(uint8_t Watermark)
{
  if (ism6hg256x_fifo_watermark_set(&reg_ctx, Watermark) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO stop on watermark
* @param  Status FIFO stop on watermark status
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Set_Stop_On_Fth(uint8_t Status)
{
  if (ism6hg256x_fifo_stop_on_wtm_set(&reg_ctx, Status) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO mode
* @param  Mode FIFO mode
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Set_Mode(uint8_t Mode)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_fifo_mode_t newMode = ISM6HG256X_BYPASS_MODE;
  switch (Mode) {
    case 0:
      newMode = ISM6HG256X_BYPASS_MODE;
      break;
    case 1:
      newMode = ISM6HG256X_FIFO_MODE;
      break;
    case 3:
      newMode = ISM6HG256X_STREAM_TO_FIFO_MODE;
      break;
    case 4:
      newMode = ISM6HG256X_BYPASS_TO_STREAM_MODE;
      break;
    case 6:
      newMode = ISM6HG256X_STREAM_MODE;
      break;
    case 7:
      newMode = ISM6HG256X_BYPASS_TO_FIFO_MODE;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  if (ret == ISM6HG256X_ERROR) {
    return ISM6HG256X_ERROR;
  }
  if (ism6hg256x_fifo_mode_set(&reg_ctx, newMode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM6HG256X FIFO tag
* @param  Tag FIFO tag
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Get_Tag(uint8_t *Tag)
{
  ism6hg256x_fifo_data_out_tag_t tag_local;
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FIFO_DATA_OUT_TAG, (uint8_t *)&tag_local, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *Tag = (uint8_t)tag_local.tag_sensor;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X FIFO raw data
* @param  Data FIFO raw data array [6]
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_Get_Data(uint8_t *Data)
{
  if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_FIFO_DATA_OUT_X_L, Data, 6) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X FIFO accelero single sample (16-bit data per 3 axes) and calculate acceleration [mg]
* @param  Acceleration FIFO accelero axes [mg]
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_X_Get_Axes(ISM6HG256X_Axes_t *Acceleration)
{
  ism6hg256x_axis3bit16_t data_raw;
  float_t sensitivity = 0.0f;
  float_t acceleration_float_t[3];
  if (FIFO_Get_Data(data_raw.u8bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (Get_X_Sensitivity(&sensitivity) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  acceleration_float_t[0] = (float_t)data_raw.i16bit[0] * sensitivity;
  acceleration_float_t[1] = (float_t)data_raw.i16bit[1] * sensitivity;
  acceleration_float_t[2] = (float_t)data_raw.i16bit[2] * sensitivity;
  Acceleration->x = (int32_t)acceleration_float_t[0];
  Acceleration->y = (int32_t)acceleration_float_t[1];
  Acceleration->z = (int32_t)acceleration_float_t[2];
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO accelero BDR value
* @param  Bdr FIFO accelero BDR value
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_X_Set_BDR(float_t Bdr)
{
  ism6hg256x_fifo_xl_batch_t new_bdr;
  new_bdr = (Bdr <=    0.0f) ? ISM6HG256X_XL_NOT_BATCHED
            : (Bdr <=    1.8f) ? ISM6HG256X_XL_BATCHED_AT_1Hz875
            : (Bdr <=    7.5f) ? ISM6HG256X_XL_BATCHED_AT_7Hz5
            : (Bdr <=   15.0f) ? ISM6HG256X_XL_BATCHED_AT_15Hz
            : (Bdr <=   30.0f) ? ISM6HG256X_XL_BATCHED_AT_30Hz
            : (Bdr <=   60.0f) ? ISM6HG256X_XL_BATCHED_AT_60Hz
            : (Bdr <=  120.0f) ? ISM6HG256X_XL_BATCHED_AT_120Hz
            : (Bdr <=  240.0f) ? ISM6HG256X_XL_BATCHED_AT_240Hz
            : (Bdr <=  480.0f) ? ISM6HG256X_XL_BATCHED_AT_480Hz
            : (Bdr <=  960.0f) ? ISM6HG256X_XL_BATCHED_AT_960Hz
            : (Bdr <= 1920.0f) ? ISM6HG256X_XL_BATCHED_AT_1920Hz
            : (Bdr <= 3840.0f) ? ISM6HG256X_XL_BATCHED_AT_3840Hz
            :                    ISM6HG256X_XL_BATCHED_AT_7680Hz;
  if (ism6hg256x_fifo_xl_batch_set(&reg_ctx, new_bdr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X FIFO gyro single sample (16-bit data per 3 axes) and calculate angular velocity [mDPS]
* @param  AngularVelocity FIFO gyro axes [mDPS]
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_G_Get_Axes(ISM6HG256X_Axes_t *AngularVelocity)
{
  ism6hg256x_axis3bit16_t data_raw;
  float_t sensitivity = 0.0f;
  float_t angular_velocity_float_t[3];
  if (FIFO_Get_Data(data_raw.u8bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  if (Get_G_Sensitivity(&sensitivity) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  angular_velocity_float_t[0] = (float_t)data_raw.i16bit[0] * sensitivity;
  angular_velocity_float_t[1] = (float_t)data_raw.i16bit[1] * sensitivity;
  angular_velocity_float_t[2] = (float_t)data_raw.i16bit[2] * sensitivity;
  AngularVelocity->x = (int32_t)angular_velocity_float_t[0];
  AngularVelocity->y = (int32_t)angular_velocity_float_t[1];
  AngularVelocity->z = (int32_t)angular_velocity_float_t[2];
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X FIFO gyro BDR value
* @param  Bdr FIFO gyro BDR value
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::FIFO_G_Set_BDR(float_t Bdr)
{
  ism6hg256x_fifo_gy_batch_t new_bdr;
  new_bdr = (Bdr <=    0.0f) ? ISM6HG256X_GY_NOT_BATCHED
            : (Bdr <=    1.8f) ? ISM6HG256X_GY_BATCHED_AT_1Hz875
            : (Bdr <=    7.5f) ? ISM6HG256X_GY_BATCHED_AT_7Hz5
            : (Bdr <=   15.0f) ? ISM6HG256X_GY_BATCHED_AT_15Hz
            : (Bdr <=   30.0f) ? ISM6HG256X_GY_BATCHED_AT_30Hz
            : (Bdr <=   60.0f) ? ISM6HG256X_GY_BATCHED_AT_60Hz
            : (Bdr <=  120.0f) ? ISM6HG256X_GY_BATCHED_AT_120Hz
            : (Bdr <=  240.0f) ? ISM6HG256X_GY_BATCHED_AT_240Hz
            : (Bdr <=  480.0f) ? ISM6HG256X_GY_BATCHED_AT_480Hz
            : (Bdr <=  960.0f) ? ISM6HG256X_GY_BATCHED_AT_960Hz
            : (Bdr <= 1920.0f) ? ISM6HG256X_GY_BATCHED_AT_1920Hz
            : (Bdr <= 3840.0f) ? ISM6HG256X_GY_BATCHED_AT_3840Hz
            :                    ISM6HG256X_GY_BATCHED_AT_7680Hz;
  if (ism6hg256x_fifo_gy_batch_set(&reg_ctx, new_bdr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Enable the ISM6HG256X gyroscope sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U) {
    return ISM6HG256X_OK;
  }
  /* Output data rate selection. */
  if (Set_G_OutputDataRate(gyro_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  gyro_is_enabled = 1;
  return ISM6HG256X_OK;
}
/**
* @brief  Disable the ISM6HG256X gyroscope sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    return ISM6HG256X_OK;
  }
  /* Get current output data rate. */
  if (Get_G_OutputDataRate(&gyro_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Output data rate selection - power down. */
  if (ism6hg256x_gy_data_rate_set(&reg_ctx, ISM6HG256X_ODR_OFF) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  gyro_is_enabled = 0;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X gyroscope sensor sensitivity
* @param  Sensitivity pointer
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_G_Sensitivity(float_t *Sensitivity)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_gy_full_scale_t full_scale;
  /* Read actual full scale selection from sensor. */
  if (ism6hg256x_gy_full_scale_get(&reg_ctx, &full_scale) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case ISM6HG256X_250dps:
      *Sensitivity = ISM6HG256X_GYRO_SENSITIVITY_FS_250DPS;
      break;
    case ISM6HG256X_500dps:
      *Sensitivity = ISM6HG256X_GYRO_SENSITIVITY_FS_500DPS;
      break;
    case ISM6HG256X_1000dps:
      *Sensitivity = ISM6HG256X_GYRO_SENSITIVITY_FS_1000DPS;
      break;
    case ISM6HG256X_2000dps:
      *Sensitivity = ISM6HG256X_GYRO_SENSITIVITY_FS_2000DPS;
      break;
    case ISM6HG256X_4000dps:
      *Sensitivity = ISM6HG256X_GYRO_SENSITIVITY_FS_4000DPS;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Get the ISM6HG256X gyroscope sensor output data rate
* @param  Odr pointer where the output data rate is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_G_OutputDataRate(float_t *Odr)
{
  ism6hg256x_data_rate_t odr_low_level;
  /* Get current output data rate. */
  if (ism6hg256x_gy_data_rate_get(&reg_ctx, &odr_low_level) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  if (ism6hg256x_data_rate_to_float(odr_low_level, Odr) != 0) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X gyroscope sensor output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_G_OutputDataRate(float_t Odr)
{
  if (gyro_is_enabled == 1U) {
    return Set_G_OutputDataRate_When_Enabled(Odr);
  } else {
    return Set_G_OutputDataRate_When_Disabled(Odr);
  }
}
/**
* @brief  Set the ISM6HG256X gyroscope sensor output data rate with operating mode
* @param  Odr the output data rate value to be set
* @param  Mode the gyroscope operating mode
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_G_OutputDataRate_With_Mode(float_t Odr,
                                                                         ISM6HG256X_GYRO_Operating_Mode_t Mode)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  float_t odr_new = 0.0f;
  switch (Mode) {
    case ISM6HG256X_GYRO_HIGH_PERFORMANCE_MODE:
      if (ism6hg256x_gy_mode_set(&reg_ctx, ISM6HG256X_GY_HIGH_PERFORMANCE_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
      odr_new = (Odr <    7.5f) ?    7.5f
                : (Odr > 7680.0f) ? 7680.0f
                :                       Odr;
      break;
    case ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE:
      ret = ISM6HG256X_ERROR;
      break;
    case ISM6HG256X_GYRO_ODR_TRIGGERED_MODE:
      ret = ISM6HG256X_ERROR;
      break;
    case ISM6HG256X_GYRO_SLEEP_MODE:
      ret = ISM6HG256X_ERROR;
      break;
    case ISM6HG256X_GYRO_LOW_POWER_MODE:
      if (ism6hg256x_gy_mode_set(&reg_ctx, ISM6HG256X_GY_LOW_POWER_MD) != ISM6HG256X_OK) {
        return ISM6HG256X_ERROR;
      }
      /* Valid ODR: 7.5Hz <= Odr <= 240kHz */
      odr_new = (Odr <   7.5f) ?   7.5f
                : (Odr > 240.0f) ? 240.0f
                :                     Odr;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  if (ret != ISM6HG256X_OK) {
    return ret;
  }
  if (gyro_is_enabled == 1U) {
    ret = Set_G_OutputDataRate_When_Enabled(odr_new);
  } else {
    ret = Set_G_OutputDataRate_When_Disabled(odr_new);
  }
  return ret;
}
/**
* @brief  Get the ISM6HG256X gyroscope sensor full scale
* @param  FullScale pointer where the full scale is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_G_FullScale(int32_t  *FullScale)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_gy_full_scale_t fs_low_level;
  /* Read actual full scale selection from sensor. */
  if (ism6hg256x_gy_full_scale_get(&reg_ctx, &fs_low_level) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  switch (fs_low_level) {
    case ISM6HG256X_250dps:
      *FullScale =  250;
      break;
    case ISM6HG256X_500dps:
      *FullScale =  500;
      break;
    case ISM6HG256X_1000dps:
      *FullScale = 1000;
      break;
    case ISM6HG256X_2000dps:
      *FullScale = 2000;
      break;
    case ISM6HG256X_4000dps:
      *FullScale = 4000;
      break;
    default:
      ret = ISM6HG256X_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM6HG256X gyroscope sensor full scale
* @param  FullScale the functional full scale to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_G_FullScale(int32_t FullScale)
{
  ism6hg256x_gy_full_scale_t new_fs;
  new_fs = (FullScale <=  250) ? ISM6HG256X_250dps
           : (FullScale <=  500) ? ISM6HG256X_500dps
           : (FullScale <= 1000) ? ISM6HG256X_1000dps
           : (FullScale <= 2000) ? ISM6HG256X_2000dps
           :                       ISM6HG256X_4000dps;
  if (ism6hg256x_gy_full_scale_set(&reg_ctx, new_fs) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X gyroscope sensor raw axes
* @param  Value pointer where the raw values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_G_AxesRaw(ISM6HG256X_AxesRaw_t *Value)
{
  ism6hg256x_axis3bit16_t data_raw;
  /* Read raw data values. */
  if (ism6hg256x_angular_rate_raw_get(&reg_ctx, data_raw.i16bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Format the data. */
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X gyroscope sensor axes
* @param  AngularRate pointer where the values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_G_Axes(ISM6HG256X_Axes_t *AngularRate)
{
  ism6hg256x_axis3bit16_t data_raw;
  float_t sensitivity = 0.0f;
  /* Read raw data values. */
  if (ism6hg256x_angular_rate_raw_get(&reg_ctx, data_raw.i16bit) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Get ISM6HG256X actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  /* Calculate the data. */
  AngularRate->x = (int32_t)((float_t)((float_t)data_raw.i16bit[0] * sensitivity));
  AngularRate->y = (int32_t)((float_t)((float_t)data_raw.i16bit[1] * sensitivity));
  AngularRate->z = (int32_t)((float_t)((float_t)data_raw.i16bit[2] * sensitivity));
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X register value
* @param  Reg address to be read
* @param  Data pointer where the value is written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (ism6hg256x_read_reg(&reg_ctx, Reg, Data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X register value
* @param  Reg address to be written
* @param  Data value to be written
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (ism6hg256x_write_reg(&reg_ctx, Reg, &Data, 1) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X ACC data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  ism6hg256x_data_ready_t val;
  if (ism6hg256x_flag_data_ready_get(&reg_ctx, &val) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *Status = val.drdy_xl;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X high-g ACC data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::HG_X_Get_DRDY_Status(uint8_t *Status)
{
  ism6hg256x_data_ready_t val;
  if (ism6hg256x_flag_data_ready_get(&reg_ctx, &val) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *Status = val.drdy_hgxl;
  return ISM6HG256X_OK;
}
/**
* @brief  Get the ISM6HG256X GYRO data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  ism6hg256x_data_ready_t val;
  if (ism6hg256x_flag_data_ready_get(&reg_ctx, &val) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  *Status = val.drdy_gy;
  return ISM6HG256X_OK;
}
/**
* @brief  Set memory bank
* @param  Val the value of memory bank in reg FUNC_CFG_ACCESS
*         0 - ISM6HG256X_MAIN_MEM_BANK, 1 - ISM6HG256X_EMBED_FUNC_MEM_BANK, 2 - ISM6HG256X_SENSOR_HUB_MEM_BANK
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_Mem_Bank(uint8_t Val)
{
  ISM6HG256XStatusTypeDef ret = ISM6HG256X_OK;
  ism6hg256x_mem_bank_t reg;
  reg = (Val == 1U) ? ISM6HG256X_EMBED_FUNC_MEM_BANK
        : (Val == 2U) ? ISM6HG256X_SENSOR_HUB_MEM_BANK
        :               ISM6HG256X_MAIN_MEM_BANK;
  if (ism6hg256x_mem_bank_set(&reg_ctx, reg) != ISM6HG256X_OK) {
    ret = ISM6HG256X_ERROR;
  }
  return ret;
}
/**
  * @}
  */

/**
* @brief  Set the ISM6HG256X accelerometer power mode
* @param  PowerMode Value of the powerMode
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_Power_Mode(uint8_t PowerMode)
{
  ism6hg256x_xl_mode_t new_mode;
  new_mode = (PowerMode == 0x00U) ? ISM6HG256X_XL_HIGH_PERFORMANCE_MD
             : (PowerMode == 0x01U) ? ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD
             : (PowerMode == 0x03U) ? ISM6HG256X_XL_ODR_TRIGGERED_MD
             : (PowerMode == 0x04U) ? ISM6HG256X_XL_LOW_POWER_2_AVG_MD
             : (PowerMode == 0x05U) ? ISM6HG256X_XL_LOW_POWER_4_AVG_MD
             : (PowerMode == 0x06U) ? ISM6HG256X_XL_LOW_POWER_8_AVG_MD
             :                        ISM6HG256X_XL_NORMAL_MD;
  if (ism6hg256x_xl_mode_set(&reg_ctx, new_mode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X gyroscope power mode
* @param  PowerMode Value of the powerMode
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_G_Power_Mode(uint8_t PowerMode)
{
  ism6hg256x_gy_mode_t new_mode;
  new_mode = (PowerMode == 0x00U) ? ISM6HG256X_GY_HIGH_PERFORMANCE_MD
             : (PowerMode == 0x01U) ? ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD
             : (PowerMode == 0x03U) ? ISM6HG256X_GY_ODR_TRIGGERED_MD
             : (PowerMode == 0x04U) ? ISM6HG256X_GY_SLEEP_MD
             :                        ISM6HG256X_GY_LOW_POWER_MD;
  if (ism6hg256x_gy_mode_set(&reg_ctx, new_mode) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }
  return ISM6HG256X_OK;
}
/**
* @brief  Set the ISM6HG256X accelerometer filter mode
* @param  LowHighPassFlag 0/1 for setting low-pass/high-pass filter mode
* @param  FilterMode Value of the filter Mode
* @retval 0 in case of success, an error code otherwise
*/
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode)
{
  ism6hg256x_filt_xl_lp2_bandwidth_t new_mode;
  new_mode = (FilterMode == 0x00U) ? ISM6HG256X_XL_ULTRA_LIGHT
             : (FilterMode == 0x01U) ? ISM6HG256X_XL_VERY_LIGHT
             : (FilterMode == 0x02U) ? ISM6HG256X_XL_LIGHT
             : (FilterMode == 0x03U) ? ISM6HG256X_XL_MEDIUM
             : (FilterMode == 0x04U) ? ISM6HG256X_XL_STRONG
             : (FilterMode == 0x05U) ? ISM6HG256X_XL_VERY_STRONG
             : (FilterMode == 0x06U) ? ISM6HG256X_XL_AGGRESSIVE
             :                         ISM6HG256X_XL_XTREME;
  if (LowHighPassFlag == 0U) {
    /*Set accelerometer low_pass filter-mode*/
    /*Set to 1 LPF2 bit (CTRL8_XL)*/
    if (ism6hg256x_filt_xl_lp2_set(&reg_ctx, 1) != ISM6HG256X_OK) {
      return ISM6HG256X_ERROR;
    }
    if (ism6hg256x_filt_xl_lp2_bandwidth_set(&reg_ctx, new_mode) != ISM6HG256X_OK) {
      return ISM6HG256X_ERROR;
    }
  } else {
    if (ism6hg256x_filt_xl_lp2_set(&reg_ctx, 0) != ISM6HG256X_OK) {
      return ISM6HG256X_ERROR;
    }
    /*Set accelerometer high_pass filter-mode*/
    if (ism6hg256x_filt_xl_lp2_bandwidth_set(&reg_ctx, new_mode) != ISM6HG256X_OK) {
      return ISM6HG256X_ERROR;
    }
  }
  return ISM6HG256X_OK;
}
/**
  * @brief  Set the ISM6HG256X accelerometer sensor output data rate when enabled
  * @param  pObj the device pObj
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_OutputDataRate_When_Enabled(float_t Odr)
{
  ism6hg256x_data_rate_t new_odr;
  uint8_t haodr_cfg = 0;

  if (acc_mode == ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE) {
    if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_HAODR_CFG, &haodr_cfg, 1) != 0) {
      return ISM6HG256X_ERROR;
    }
  }

  new_odr = ism6hg256x_float_to_data_rate(haodr_cfg, Odr);

  /* Output data rate selection. */
  if (ism6hg256x_xl_data_rate_set(&reg_ctx, new_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}

/**
  * @brief  Set the ISM6HG256X accelerometer sensor output data rate when disabled
  * @param  pObj the device pObj
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_OutputDataRate_When_Disabled(float_t Odr)
{
  uint8_t haodr_cfg = 0;

  if (acc_mode == ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE) {
    if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_HAODR_CFG, &haodr_cfg, 1) != 0) {
      return ISM6HG256X_ERROR;
    }
  }

  acc_odr = ism6hg256x_float_to_data_rate(haodr_cfg, Odr);

  return ISM6HG256X_OK;
}

/**
  * @brief  Set the ISM6HG256X high-g accelerometer sensor output data rate when enabled
  * @param  pObj the device pObj
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_HG_OutputDataRate_When_Enabled(float_t Odr)
{
  ism6hg256x_hg_xl_data_rate_t new_odr;
  uint8_t haodr_cfg = 0;

  /* per application note: high-g accelerometer automatically enters high-accuracy mode when both gyro and accel are configured to it */
  if (acc_mode == ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE && gyro_mode == ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE) {
    if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_HAODR_CFG, &haodr_cfg, 1) != 0) {
      return ISM6HG256X_ERROR;
    }
  }

  new_odr = ism6hg256x_hg_float_to_data_rate(haodr_cfg, acc_hg_odr);

  /* Output data rate selection. */
  if (ism6hg256x_hg_xl_data_rate_set(&reg_ctx, new_odr, PROPERTY_ENABLE) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}

/**
  * @brief  Set the ISM6HG256X high-g accelerometer sensor output data rate when disabled
  * @param  pObj the device pObj
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_X_HG_OutputDataRate_When_Disabled(float_t Odr)
{
  acc_hg_odr = Odr;

  return ISM6HG256X_OK;
}
/**
  * @brief  Set the ISM6HG256X gyroscope sensor output data rate when enabled
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_G_OutputDataRate_When_Enabled(float_t Odr)
{
  ism6hg256x_data_rate_t new_odr;
  uint8_t haodr_cfg = 0;

  if (gyro_mode == ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE) {
    if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_HAODR_CFG, &haodr_cfg, 1) != 0) {
      return ISM6HG256X_ERROR;
    }
  }

  new_odr = ism6hg256x_float_to_data_rate(haodr_cfg, Odr);

  /* Output data rate selection. */
  if (ism6hg256x_gy_data_rate_set(&reg_ctx, new_odr) != ISM6HG256X_OK) {
    return ISM6HG256X_ERROR;
  }

  return ISM6HG256X_OK;
}

/**
  * @brief  Set the ISM6HG256X gyroscope sensor output data rate when disabled
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
ISM6HG256XStatusTypeDef ISM6HG256XSensor::Set_G_OutputDataRate_When_Disabled(float_t Odr)
{
  uint8_t haodr_cfg = 0;

  if (gyro_mode == ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE) {
    if (ism6hg256x_read_reg(&reg_ctx, ISM6HG256X_HAODR_CFG, &haodr_cfg, 1) != 0) {
      return ISM6HG256X_ERROR;
    }
  }

  gyro_odr = ism6hg256x_float_to_data_rate(haodr_cfg, Odr);

  return ISM6HG256X_OK;
}
int32_t ISM6HG256X_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((ISM6HG256XSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}
int32_t ISM6HG256X_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((ISM6HG256XSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}

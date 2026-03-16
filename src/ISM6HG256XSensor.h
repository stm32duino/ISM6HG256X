/**
 ******************************************************************************
 * @file    ISM6HG256XSensor.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    05 August 2025
 * @brief   Abstract Class of a ISM6HG256X sensor.
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
/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __ISM6HG256XSensor_H__
#define __ISM6HG256XSensor_H__
/* Includes ------------------------------------------------------------------*/
/* For compatibility with ESP32 platforms */
#ifdef ESP32
  #ifndef MSBFIRST
    #define MSBFIRST SPI_MSBFIRST
  #endif
#endif
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "ism6hg256x_reg.h"
/* Defines -------------------------------------------------------------------*/
#define ISM6HG256X_ACC_SENSITIVITY_FS_2G    0.061f
#define ISM6HG256X_ACC_SENSITIVITY_FS_4G    0.122f
#define ISM6HG256X_ACC_SENSITIVITY_FS_8G    0.244f
#define ISM6HG256X_ACC_SENSITIVITY_FS_16G   0.488f
#define ISM6HG256X_ACC_SENSITIVITY_FS_32G   0.976f
#define ISM6HG256X_ACC_SENSITIVITY_FS_64G   1.952f
#define ISM6HG256X_ACC_SENSITIVITY_FS_128G  3.904f
#define ISM6HG256X_ACC_SENSITIVITY_FS_256G  7.808f
#define ISM6HG256X_GYRO_SENSITIVITY_FS_250DPS     8.750f
#define ISM6HG256X_GYRO_SENSITIVITY_FS_500DPS    17.500f
#define ISM6HG256X_GYRO_SENSITIVITY_FS_1000DPS   35.000f
#define ISM6HG256X_GYRO_SENSITIVITY_FS_2000DPS   70.000f
#define ISM6HG256X_GYRO_SENSITIVITY_FS_4000DPS  140.000f
/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  ISM6HG256X_OK = 0,
  ISM6HG256X_ERROR = -1
} ISM6HG256XStatusTypeDef;
typedef enum {
  ISM6HG256X_INT1_PIN,
  ISM6HG256X_INT2_PIN,
} ISM6HG256X_SensorIntPin_t;
typedef enum {
  ISM6HG256X_ACC_HIGH_PERFORMANCE_MODE,
  ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE,
  ISM6HG256X_ACC_ODR_TRIGGERED_MODE,
  ISM6HG256X_ACC_LOW_POWER_MODE1,
  ISM6HG256X_ACC_LOW_POWER_MODE2,
  ISM6HG256X_ACC_LOW_POWER_MODE3,
  ISM6HG256X_ACC_NORMAL_MODE,
} ISM6HG256X_ACC_Operating_Mode_t;
typedef enum {
  ISM6HG256X_GYRO_HIGH_PERFORMANCE_MODE,
  ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE,
  ISM6HG256X_GYRO_ODR_TRIGGERED_MODE,
  ISM6HG256X_GYRO_SLEEP_MODE,
  ISM6HG256X_GYRO_LOW_POWER_MODE,
} ISM6HG256X_GYRO_Operating_Mode_t;
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} ism6hg256x_axis3bit16_t;
typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} ism6hg256x_axis1bit16_t;
typedef union {
  int32_t i32bit[3];
  uint8_t u8bit[12];
} ism6hg256x_axis3bit32_t;
typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} ism6hg256x_axis1bit32_t;
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} ISM6HG256X_AxesRaw_t;
typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} ISM6HG256X_Axes_t;
typedef struct {
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} ISM6HG256X_Event_Status_t;
/* Class Declaration ---------------------------------------------------------*/
/**
 * Abstract class of a ISM6HG256X pressure sensor.
 */
class ISM6HG256XSensor {
  public:
    ISM6HG256XSensor(TwoWire *i2c, uint8_t address = ISM6HG256X_I2C_ADD_L);
    ISM6HG256XSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);
    ISM6HG256XStatusTypeDef begin();
    ISM6HG256XStatusTypeDef end();
    ISM6HG256XStatusTypeDef ReadID(uint8_t *Id);
    ISM6HG256XStatusTypeDef Enable_High_Accuracy_Mode(void);
    ISM6HG256XStatusTypeDef Set_High_Accuracy_ODR_Range(ism6hg256x_haodr_sel_t sel);
    ISM6HG256XStatusTypeDef Enable_X();
    ISM6HG256XStatusTypeDef Disable_X();
    ISM6HG256XStatusTypeDef Get_X_Sensitivity(float_t *Sensitivity);
    ISM6HG256XStatusTypeDef Get_X_OutputDataRate(float_t *Odr);
    ISM6HG256XStatusTypeDef Set_X_OutputDataRate(float_t Odr);
    ISM6HG256XStatusTypeDef Set_X_OutputDataRate_With_Mode(float_t Odr, ISM6HG256X_ACC_Operating_Mode_t Mode);
    ISM6HG256XStatusTypeDef Get_X_FullScale(int32_t *FullScale);
    ISM6HG256XStatusTypeDef Set_X_FullScale(int32_t FullScale);
    ISM6HG256XStatusTypeDef Get_X_AxesRaw(ISM6HG256X_AxesRaw_t *Value);
    ISM6HG256XStatusTypeDef Get_X_Axes(ISM6HG256X_Axes_t *Acceleration);
    ISM6HG256XStatusTypeDef Get_X_Event_Status(ISM6HG256X_Event_Status_t *Status);
    ISM6HG256XStatusTypeDef Enable_Free_Fall_Detection(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_Free_Fall_Detection();
    ISM6HG256XStatusTypeDef Set_Free_Fall_Threshold(uint8_t Threshold);
    ISM6HG256XStatusTypeDef Set_Free_Fall_Duration(uint8_t Duration);
    ISM6HG256XStatusTypeDef Enable_Wake_Up_Detection(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_Wake_Up_Detection();
    ISM6HG256XStatusTypeDef Set_Wake_Up_Threshold(uint32_t Threshold, uint32_t InactivityThreshold);
    ISM6HG256XStatusTypeDef Set_Wake_Up_Duration(uint8_t Duration);
    ISM6HG256XStatusTypeDef Enable_Single_Tap_Detection(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_Single_Tap_Detection();
    ISM6HG256XStatusTypeDef Enable_Double_Tap_Detection(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_Double_Tap_Detection();
    ISM6HG256XStatusTypeDef Set_Tap_Threshold(uint8_t Threshold);
    ISM6HG256XStatusTypeDef Set_Tap_Shock_Time(uint8_t Time);
    ISM6HG256XStatusTypeDef Set_Tap_Quiet_Time(uint8_t Time);
    ISM6HG256XStatusTypeDef Set_Tap_Duration_Time(uint8_t Time);
    ISM6HG256XStatusTypeDef Enable_6D_Orientation(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_6D_Orientation();
    ISM6HG256XStatusTypeDef Set_6D_Orientation_Threshold(uint8_t Threshold);
    ISM6HG256XStatusTypeDef Get_6D_Orientation_XL(uint8_t *XLow);
    ISM6HG256XStatusTypeDef Get_6D_Orientation_XH(uint8_t *XHigh);
    ISM6HG256XStatusTypeDef Get_6D_Orientation_YL(uint8_t *YLow);
    ISM6HG256XStatusTypeDef Get_6D_Orientation_YH(uint8_t *YHigh);
    ISM6HG256XStatusTypeDef Get_6D_Orientation_ZL(uint8_t *ZLow);
    ISM6HG256XStatusTypeDef Get_6D_Orientation_ZH(uint8_t *ZHigh);
    ISM6HG256XStatusTypeDef Enable_Tilt_Detection(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_Tilt_Detection();
    ISM6HG256XStatusTypeDef Enable_Pedometer(ISM6HG256X_SensorIntPin_t IntPin);
    ISM6HG256XStatusTypeDef Disable_Pedometer();
    ISM6HG256XStatusTypeDef Get_Step_Count(uint16_t *StepCount);
    ISM6HG256XStatusTypeDef Step_Counter_Reset();
    ISM6HG256XStatusTypeDef Enable_HG_X();
    ISM6HG256XStatusTypeDef Disable_HG_X();
    ISM6HG256XStatusTypeDef Get_HG_X_Sensitivity(float_t *Sensitivity);
    ISM6HG256XStatusTypeDef Get_HG_X_OutputDataRate(float_t *Odr);
    ISM6HG256XStatusTypeDef Set_HG_X_OutputDataRate(float_t Odr);
    ISM6HG256XStatusTypeDef HG_X_GetFullScale(int32_t *FullScale);
    ISM6HG256XStatusTypeDef Set_HG_X_FullScale(int32_t FullScale);
    ISM6HG256XStatusTypeDef Get_HG_X_AxesRaw(ISM6HG256X_AxesRaw_t *Value);
    ISM6HG256XStatusTypeDef HG_X_GetAxes(ISM6HG256X_Axes_t *Acceleration);
    ISM6HG256XStatusTypeDef FIFO_Get_Num_Samples(uint16_t *NumSamples);
    ISM6HG256XStatusTypeDef FIFO_Get_Full_Status(uint8_t *Status);
    ISM6HG256XStatusTypeDef FIFO_Set_INT1_FIFO_Full(uint8_t Status);
    ISM6HG256XStatusTypeDef FIFO_Set_INT2_FIFO_Full(uint8_t Status);
    ISM6HG256XStatusTypeDef FIFO_Set_Watermark_Level(uint8_t Watermark);
    ISM6HG256XStatusTypeDef FIFO_Set_Stop_On_Fth(uint8_t Status);
    ISM6HG256XStatusTypeDef FIFO_Set_Mode(uint8_t Mode);
    ISM6HG256XStatusTypeDef FIFO_Get_Tag(uint8_t *Tag);
    ISM6HG256XStatusTypeDef FIFO_Get_Data(uint8_t *Data);
    ISM6HG256XStatusTypeDef FIFO_X_Get_Axes(ISM6HG256X_Axes_t *Acceleration);
    ISM6HG256XStatusTypeDef FIFO_X_Set_BDR(float_t Bdr);
    ISM6HG256XStatusTypeDef FIFO_G_Get_Axes(ISM6HG256X_Axes_t *AngularVelocity);
    ISM6HG256XStatusTypeDef FIFO_G_Set_BDR(float_t Bdr);
    ISM6HG256XStatusTypeDef Enable_G();
    ISM6HG256XStatusTypeDef Disable_G();
    ISM6HG256XStatusTypeDef Get_G_Sensitivity(float_t *Sensitivity);
    ISM6HG256XStatusTypeDef Get_G_OutputDataRate(float_t *Odr);
    ISM6HG256XStatusTypeDef Set_G_OutputDataRate(float_t Odr);
    ISM6HG256XStatusTypeDef Set_G_OutputDataRate_With_Mode(float_t Odr, ISM6HG256X_GYRO_Operating_Mode_t Mode);
    ISM6HG256XStatusTypeDef Get_G_FullScale(int32_t *FullScale);
    ISM6HG256XStatusTypeDef Set_G_FullScale(int32_t FullScale);
    ISM6HG256XStatusTypeDef Get_G_AxesRaw(ISM6HG256X_AxesRaw_t *Value);
    ISM6HG256XStatusTypeDef Get_G_Axes(ISM6HG256X_Axes_t *AngularRate);
    ISM6HG256XStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    ISM6HG256XStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    ISM6HG256XStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    ISM6HG256XStatusTypeDef HG_X_Get_DRDY_Status(uint8_t *Status);
    ISM6HG256XStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    ISM6HG256XStatusTypeDef Set_X_Power_Mode(uint8_t PowerMode);
    ISM6HG256XStatusTypeDef Set_G_Power_Mode(uint8_t PowerMode);
    ISM6HG256XStatusTypeDef Set_X_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode);
    ISM6HG256XStatusTypeDef Set_G_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode);
    ISM6HG256XStatusTypeDef Set_Mem_Bank(uint8_t Val);
    /**
      * @brief Utility function to read data.
      * @param  pBuffer: pointer to data to be read.
      * @param  RegisterAddr: specifies internal address register to be read.
      * @param  NumByteToRead: number of bytes to be read.
      * @retval 0 if ok, an error code otherwise.
      */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(cs_pin, LOW);
        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }
        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
        return 0;
      }
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);
        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);
        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }
        return 0;
      }
      return 1;
    }
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(cs_pin, LOW);
        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }
        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
        return 0;
      }
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }
        dev_i2c->endTransmission(true);
        return 0;
      }
      return 1;
    }
  private:
    ISM6HG256XStatusTypeDef Set_X_OutputDataRate_When_Enabled(float_t Odr);
    ISM6HG256XStatusTypeDef Set_X_OutputDataRate_When_Disabled(float_t Odr);
    ISM6HG256XStatusTypeDef Set_X_HG_OutputDataRate_When_Enabled(float_t Odr);
    ISM6HG256XStatusTypeDef Set_X_HG_OutputDataRate_When_Disabled(float_t Odr);
    ISM6HG256XStatusTypeDef Set_G_OutputDataRate_When_Enabled(float_t Odr);
    ISM6HG256XStatusTypeDef Set_G_OutputDataRate_When_Disabled(float_t Odr);
    /* Helper classes. */
    TwoWire  *dev_i2c;
    SPIClass *dev_spi;
    /* Configuration */
    uint8_t  address;
    int      cs_pin;
    uint32_t spi_speed;
    uint8_t                           is_initialized;
    uint8_t                           acc_is_enabled;
    uint8_t                           acc_hg_is_enabled;
    uint8_t                           gyro_is_enabled;
    ISM6HG256X_GYRO_Operating_Mode_t  gyro_mode;
    ISM6HG256X_ACC_Operating_Mode_t   acc_mode;
    float_t                           acc_odr;
    float_t                           acc_hg_odr;
    float_t                           gyro_odr;
    ism6hg256x_ctx_t reg_ctx;
};
#ifdef __cplusplus
extern "C" {
#endif
int32_t ISM6HG256X_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t ISM6HG256X_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif
#endif /* __ISM6HG256XSensor_H__ */

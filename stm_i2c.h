#ifndef _STM_I2C_H
#define _STM_I2C_H

#include "stm_core.h"

void InitI2C1_GPIO(void);
void InitI2C1(void);

typedef enum _i2c_error {
  I2C_err_ok = 0,
  I2C_err_init,
  I2C_err_read_datalength_large,
  I2C_err_address_ack,
  I2C_err_register,
  I2C_err_read_data,
  I2C_err_read_data_final,
  I2C_err_write_data,
} i2c_error;

i2c_error I2C1_ReadByte(uint8_t addr, uint8_t reg, uint8_t *result);
i2c_error I2C1_ReadByteNR(uint8_t addr, uint8_t reg, uint8_t *result);
i2c_error I2C1_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *result, int len);
i2c_error I2C1_WriteByte(uint8_t addr, uint8_t reg, uint8_t value);

#endif // _STM_I2C_H

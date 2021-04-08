#include "stm_i2c.h"

void InitI2C1_GPIO(void)
{
  Nucleo_SetPinGPIO(GPIOB, 8, ioPortAlternatePP);
  Nucleo_SetAFGPIO(GPIOB, 8, 4);      // AF04 = I2C1_SCL
  Nucleo_SetPinGPIO(GPIOB, 9, ioPortAlternateOC); // !!! OPEN DRAIN
  Nucleo_SetAFGPIO(GPIOB, 9, 4);      // AF04 = I2C1_SDA
}

void InitI2C1(void)
{
  uint32_t spd = 100000;

  if (!(RCC->APB1ENR1 & RCC_APB1ENR1_I2C1EN))
  {
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
  }
  
  // I2C_Reset();
  {
    uint32_t tout;
    
    I2C1->CR1 = I2C_CR1_SWRST;      // reset peripheral signal
    for (tout = 100; tout; tout--)  // short delay
      asm("NOP");
    I2C1->CR1 = 0;
  }

  // configuration
  I2C1->CR1 = I2C_CR1_PE;         // enable peripheral, remainnig bits = 0
  I2C1->CR2 = 0;                  // clear all cfg. bits
  
#ifdef STM32F411xE
  {
    int apbClk;
    {
      int apb1div = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;  // u F411 je to bit 10..12

      if ((apb1div & 0x04) == 0) // highest bit from 3 == 0 ?
        apbClk = SystemCoreClock;  // x1, AHB = sysclock
      else
        apbClk = SystemCoreClock >> ((apb1div & 0x03) + 1);
    }

    int apbClkMhz = apbClk / 1000000;          // bus-clock in MHz

    I2C1->CR1 = 0;                  // disable peripheral

    // inspired by Cube generated code
    I2C1->TRISE = (spd <= 100000U)
        ? (apbClkMhz + 1U) : (((apbClkMhz * 300U) / 1000U) + 1U);

#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__)            (((((__PCLK__)/((__SPEED__) << 1U)) & I2C_CCR_CCR) < 4U)? 4U:((__PCLK__) / ((__SPEED__) << 1U)))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? ((__PCLK__) / ((__SPEED__) * 3U)) : (((__PCLK__) / ((__SPEED__) * 25U)) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__)      (((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))
#define I2C_DUTYCYCLE_2                 ((uint32_t)0x00000000U)
#define I2C_DUTYCYCLE_16_9              I2C_CCR_DUTY

    I2C1->CCR = I2C_SPEED(apbClk, spd, I2C_DUTYCYCLE_2);
  }

  #define I2C_ADDRESSINGMODE_7BIT         ((uint32_t)0x00004000)
  I2C1->OAR1 = I2C_ADDRESSINGMODE_7BIT;   // dle Cube
  
  #define I2C_DUALADDRESS_DISABLE         ((uint32_t)0x00000000)
  #define I2C_DUALADDRESS_DISABLED                I2C_DUALADDRESS_DISABLE
  I2C1->OAR2 = I2C_DUALADDRESS_DISABLED;  // dle Cube
  // end Wizard settings
#elif defined STM32WB55xx
  {
    int apb1div = GetBusClock(busClockAPB1);
  }

  I2C1->CR1 = 0;                  // disable peripheral

  //  I2C1->TIMINGR = 0xe14;

  //  I2C1->TIMEOUTR = 0;
  I2C1->TIMINGR = 0xE14;        // Cube to generuje natvrdo ??
  //! ***************************************************
  // pro 64MHz clock je   hi2c1.Init.Timing = 0x10707DBC;
  //! ***************************************************

  I2C1->OAR1 &= ~I2C_OAR1_OA1EN;

#define I2C_ADDRESSINGMODE_7BIT         (0x00000001U)
#define I2C_ADDRESSINGMODE_10BIT        (0x00000002U)
#define I2C_DUALADDRESS_DISABLE         (0x00000000U)
#define I2C_DUALADDRESS_ENABLE          I2C_OAR2_OA2EN

  I2C1->OAR1 = (I2C_OAR1_OA1EN | 0);    // OwnAddress1

  I2C1->OAR2 &= ~I2C_DUALADDRESS_ENABLE;
  I2C1->OAR2 = (0 | 0 | (0 << 8));      // DualAddressMode, OwnAddress2, OwnAddress2Masks

  I2C1->CR1 = (0 | 0);                  // GeneralCallMode, NoStretchMode

#else
#error I2C init registers - unsupported processor type !!
#endif
  
  I2C1->CR1 |= I2C_CR1_PE;        // enable peripheral
}

const uint32_t _timeoutI2C = 1000;

i2c_error I2C1_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *result, int len)
{
  uint32_t t;

  if (len > 255)
    return I2C_err_read_datalength_large;

  I2C1->CR1 = 0;
  // ? asm("DSB");
  t = _timeoutI2C / 2;
  while(I2C1->CR1 & I2C_CR1_PE)         // wait for PE = 0
  {
    if (--t == 0)
      return I2C_err_init;
  }
  I2C1->CR1 |= I2C_CR1_PE;

  //TODO wait for I2C_ISR_BUSY clear
  I2C1->CR2 = 0
      | addr
//      | ((addr > 0x7f) ? I2C_CR2_ADD10 : 0)
      | (1 << I2C_CR2_NBYTES_Pos)
      | I2C_CR2_START
      ;

  t = _timeoutI2C;
  while(!(I2C1->ISR & I2C_ISR_TXIS))
  {
    if (--t == 0)
      return I2C_err_address_ack;
  }

  I2C1->TXDR = reg;                       // adresa, odkud chci cist
  t = _timeoutI2C;
  while(!(I2C1->ISR & I2C_ISR_TC))    // aspon jeden z nich
  {
    if (--t == 0)
      return I2C_err_register;
  }

  I2C1->CR2 = 0
      | I2C_CR2_AUTOEND
      | ((len & 0xff) << I2C_CR2_NBYTES_Pos)
      | I2C_CR2_RD_WRN              // 1 = read
      | (addr & 0x1ff)
      | I2C_CR2_START
      ;

  for (; len; len--)
  {
    t = _timeoutI2C;
    while (!(I2C1->ISR & I2C_ISR_RXNE))
    {
      if (--t == 0)
        return I2C_err_read_data;
    }

    *result = I2C1->RXDR;
    result++;
  }

  t = _timeoutI2C;
  while(!(I2C1->ISR & I2C_ISR_STOPF))     // v HAL je delay, ktery ceka a pokud nedojde, ceka I2C_ISR_NACKF
  {
    if (--t == 0)
      return I2C_err_read_data_final;
  }

  I2C1->ICR = I2C_ICR_STOPCF;
  I2C1->CR2 = 0;

  return I2C_err_ok;
}

i2c_error I2C1_ReadBytesNoRestart(uint8_t addr, uint8_t reg, uint8_t *result, int len)
{
  bool restart = false;
  uint32_t t;

  if (len > 255)
    return I2C_err_read_datalength_large;

  I2C1->CR1 = 0;
  // ? asm("DSB");
  t = _timeoutI2C / 2;
  while(I2C1->CR1 & I2C_CR1_PE)         // wait for PE = 0
  {
    if (--t == 0)
      return I2C_err_init;
  }
  I2C1->CR1 |= I2C_CR1_PE;

  //TODO wait for I2C_ISR_BUSY clear
  I2C1->CR2 = 0
      | addr
//      | ((addr > 0x7f) ? I2C_CR2_ADD10 : 0)
      | (restart ? 0 : I2C_CR2_AUTOEND)
      | (1 << I2C_CR2_NBYTES_Pos)
      | I2C_CR2_START
      ;

  t = _timeoutI2C;
  while(!(I2C1->ISR & I2C_ISR_TXIS))
  {
    if (--t == 0)
      return I2C_err_address_ack;
  }
/*
  if (!restart)
    I2C1->CR2 |= I2C_CR2_STOP;      // po prenosu udela STOP
*/
  I2C1->TXDR = reg;                       // adresa, odkud chci cist
  t = _timeoutI2C;
//  while(!(I2C1->ISR & (I2C_ISR_TC | I2C_ISR_STOPF)))    // aspon jeden z nich
  while(!(I2C1->ISR & (I2C_ISR_TC | I2C_ISR_STOPF)))    // aspon jeden z nich
  {
    if (--t == 0)
      return I2C_err_register;
  }

  I2C1->CR2 = 0
      | I2C_CR2_AUTOEND
      | ((len & 0xff) << I2C_CR2_NBYTES_Pos)
      | I2C_CR2_RD_WRN              // 1 = read
      | (addr & 0x1ff)
      | I2C_CR2_START
      ;

  for (; len; len--)
  {
    t = _timeoutI2C;
    while (!(I2C1->ISR & I2C_ISR_RXNE))
    {
      if (--t == 0)
        return I2C_err_read_data;
    }

    *result = I2C1->RXDR;
    result++;
  }

  t = _timeoutI2C;
  while(!(I2C1->ISR & I2C_ISR_STOPF))     // v HAL je delay, ktery ceka a pokud nedojde, ceka I2C_ISR_NACKF
  {
    if (--t == 0)
      return I2C_err_read_data_final;
  }

  I2C1->ICR = I2C_ICR_STOPCF;
  I2C1->CR2 = 0;

  return I2C_err_ok;
}

i2c_error I2C1_ReadByte(uint8_t addr, uint8_t reg, uint8_t *result)
{
  return I2C1_ReadBytes(addr, reg, result, 1);
}

i2c_error I2C1_ReadByteNR(uint8_t addr, uint8_t reg, uint8_t *result)
{
  return I2C1_ReadBytesNoRestart(addr, reg, result, 1);
}

i2c_error I2C1_WriteByte(uint8_t addr, uint8_t reg, uint8_t value)
{
  uint32_t t;

  I2C1->CR1 = 0;
  // ? asm("DSB");
  t = _timeoutI2C / 2;
  while(I2C1->CR1 & I2C_CR1_PE)         // wait for PE = 0
  {
    if (--t == 0)
      return I2C_err_init;
  }
  I2C1->CR1 |= I2C_CR1_PE;

  //TODO wait for I2C_ISR_BUSY clear
  I2C1->CR2 = 0
      | addr
//      | ((addr > 0x7f) ? I2C_CR2_ADD10 : 0)
//      | (1 << I2C_CR2_NBYTES_Pos)
      | (2 << I2C_CR2_NBYTES_Pos)       // adresa registru + 1x data
// nee, pri zapisu neni nutny      | I2C_CR2_RELOAD
      | I2C_CR2_AUTOEND                 // naopak je treba udelat STOP
      | I2C_CR2_START
      ;

  t = _timeoutI2C;
  while (!(I2C1->ISR & I2C_ISR_TXIS))
  {
    if (--t == 0)
      return I2C_err_address_ack;
  }

  I2C1->TXDR = reg;                       // adresa, kam zapisovat
  t = _timeoutI2C;
  while (!(I2C1->ISR & I2C_ISR_TXIS))
  {
    if (--t == 0)
      return I2C_err_register;
  }

  I2C1->TXDR = value;

  t = _timeoutI2C;
  while(!(I2C1->ISR & I2C_ISR_STOPF))
  {
    if (--t == 0)
      return I2C_err_write_data;
  }

  I2C1->ICR = I2C_ICR_STOPCF;             // I2C1->CR2 |= I2C_CR2_STOP;
  I2C1->CR2 = 0;

  return I2C_err_ok;
}

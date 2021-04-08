# kei_ase_wb55_files
Zdrojové kody pro STM32WB55 používané v KEI/ASE

Zatím pracovní pro I2C, startovací verze 2021-04-07.
Předpokládá další soubory s inicializací GPIO, stdio-UART apod.

Příklad použití pro i2c_scan:

...
#include "stm_i2c.h"
...
  puts("Init I2C ");

  InitI2C1_GPIO();        // Nucleo D14 a D15 odpovida B8, B9
  InitI2C1();

  puts("Scan I2C");

  uint8_t d = 0;
  uint8_t b = 0;
  do
  {
    if ((b % 8) == 0)
    {
      if (b != 0)
        puts("");

      printf("%02x:", b);
    }

    if (b < 3)
    {
      printf(" . ");
    }
    else
    {
      if (I2C_err_ok == I2C1_ReadByte(b, 0, &d))
        printf(" %02x", b);
      else
        printf(" --");
    }
  } while(++b);           // od 0 do 7f

  puts("");

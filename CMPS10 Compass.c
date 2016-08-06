#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc177x_8x_libcfg.h"
#else
#include "lpc177x_8x_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */
#include "lpc177x_8x_i2c.h"
#include "lpc177x_8x_pinsel.h"
#include "CMPS10 Compass.h"
#include "lpc177x_8x_i2c.h"
#include "debug_frmwrk.h"
#include "lpc_types.h"

#define I2CDEV_M		(0)

__IO CPMS10_sensor cmps10;

int8_t CMPS10_ReadWrite(uint8_t* txdata, uint32_t txlen,uint8_t* rxdata, uint32_t rxlen)
{
	I2C_M_SETUP_Type i2cData;
	
	i2cData.sl_addr7bit = CMPS10_address;
	i2cData.tx_length = txlen;
  i2cData.tx_data = txdata;
  i2cData.rx_data = rxdata;
	i2cData.rx_length = rxlen;
	i2cData.retransmissions_max = 1;	
	
	if (I2C_MasterTransferData((en_I2C_unitId)I2CDEV_M, &i2cData, I2C_TRANSFER_POLLING) == SUCCESS)
	{
		return (1);
	}
	return (0);
	
}

int16_t CMPS10_RaadAll(CPMS10_sensor *sensor)
{
	unsigned char test;
	unsigned char Data[6];
	
	test = 0x02;
	if (CMPS10_ReadWrite(&test, 1, &Data[0], 4) == 0)
		return 0;
	sensor->Compass = Data[0]<<8 | Data[1];
	sensor->Pitch = Data[2];
	sensor->Roll = Data[3];

	test = 10;
	if (CMPS10_ReadWrite(&test, 1, &Data[0], 6) == 0)
		return 0;
	sensor->Raw_MagX = Data[0]<<8 | Data[1];
	sensor->Raw_MagY = Data[2]<<8 | Data[3];
	sensor->Raw_MagZ = Data[4]<<8 | Data[5];
	
	test = 16;
	if (CMPS10_ReadWrite(&test, 1, &Data[0], 6) == 0)
		return 0;
	sensor->AC_X = (float)((int16_t)(Data[0]<<8 | Data[1])) / 16384;
	sensor->AC_Y = (float)((int16_t)(Data[2]<<8 | Data[3])) / 16384;
	sensor->AC_Z = (float)((int16_t)(Data[4]<<8 | Data[5])) / 16384;
	
	
	
	return 1;
	
}


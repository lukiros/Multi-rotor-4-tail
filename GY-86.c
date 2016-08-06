
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc177x_8x_libcfg.h"
#else
#include "lpc177x_8x_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */
#include "lpc177x_8x_i2c.h"
#include "lpc177x_8x_pinsel.h"
#include "GY-86.h"
#include "lpc177x_8x_i2c.h"
#include "debug_frmwrk.h"

#define Gra 4
#define I2CDEV_M		(0)
#define oss 0

__IO MPU6050_sensor mpu6050;
__IO HMC5883_sensor hmc5883;

int8_t MPU6050_ReadWrite(uint8_t* txdata, uint32_t txlen,uint8_t* rxdata, uint32_t rxlen)
{
	I2C_M_SETUP_Type i2cData;
	
	i2cData.sl_addr7bit = MPU6050_I2C_ADDRESS;
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

int8_t HMC5883_ReadWrite(uint8_t* txdata, uint32_t txlen,uint8_t* rxdata, uint32_t rxlen)
{
	I2C_M_SETUP_Type i2cData;
	
	i2cData.sl_addr7bit = HMC5883_I2C_ADDRESS;
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


int8_t MPU6050_init(void)
{
	unsigned char Data_6050[2];
	unsigned char Re_6050;
	int t=0;
	//*********************************
	Data_6050[0] = 0x6B;
	Data_6050[1] = 0x80;
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	for(t=0; t < 200000; t++);
	//Reset MPU6050
	
	//*********************************
	Data_6050[0] = 0x6B;
	Data_6050[1] = 0x00;
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0) 
		return (0);
	//SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 0 (Internal 8MHz oscillator)
	
	//*********************************
	Data_6050[0] = 0x1A;
	Data_6050[1] = 0x00;
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	//CONFIG-- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	
	//*********************************
	Data_6050[0] = 0x1B;
	Data_6050[1] = MPU6050_FS_SEL_250;
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	//GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 250 deg/sec
	
	//*********************************
	Data_6050[0] = 0x1C;
	Data_6050[1] = MPU6050_AFS_SEL_2G;
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	//ACCEL_CONFIG  -- AFS_SEL=1 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
	
	//*********************************
	Data_6050[0] = 0x75;
	Data_6050[1] = 0x80;
	if (MPU6050_ReadWrite(&Data_6050[0], 1, &Re_6050, 1) == 0)
		return (0);
	if (Re_6050 != 0x68)
		return (0);
	//WHO AM I
	return (1);
	
}

int8_t MPU6050_ReadAll(MPU6050_sensor *ATG)
{
	unsigned char TestData[14] = {0};
	unsigned char test = MPU6050_ACCEL_XOUT_H;
	int16_t Data[7] = {0};
	int i = 0;
	
	ATG->AC_X = 0;
	ATG->AC_Y = 0;
	ATG->AC_Z = 0;
	ATG->Temp = 0;
	ATG->GY_X = 0;
	ATG->GY_Y = 0;
	ATG->GY_Z = 0;
	
	if (MPU6050_ReadWrite( &test, 1, &TestData[0], 14) == 0)
		return (0);
	
	for (i=0; i<7; i++)
	{
		Data[i] = ((TestData[(i*2)]<<8) | (TestData[(i*2)+1]));
	}
	
// 	AFS_SEL	Full Scale Range	LSB Sensitivity
// 	0				+-2g							16384 LSB/g
// 	1				+-4g							8192 LSB/g
// 	2				+-8g							4096 LSB/g
// 	3				+-16g							2048 LSB/g
// 	
// 	FS_SEL	Full Scale Range	LSB Sensitivity
// 	0				250 Dg/s					131 LSB/Dg/s
// 	1				500 Dg/s					65.5 LSB/Dg/s
// 	2				1000 Dg/s					32.8 LSB/Dg/s
// 	3				2000 Dg/s					16.4 LSB/Dg/s
	
	ATG->AC_X = ((float)Data[0] / 16384) * -1; // G unit
	ATG->AC_Y = (float)Data[1] / 16384; // G unit
	ATG->AC_Z = (float)Data[2] / 16384; // G unit
	ATG->Temp = ((float)Data[3] / 340) + 36.53; // Degrees C unit
	ATG->GY_X = (float)Data[4] / (float)131; // Dg/s unit
	ATG->GY_Y = (float)Data[5] / (float)131; // Dg/s uint
	ATG->GY_Z = (float)Data[6] / (float)131; // Dg/s uint
	
	return (1);
}

int8_t HMC5883_init(void)
{
	unsigned char Data_6050[2];
	int t =0;
// 	//at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
//     //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
//     i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
//     i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
//     i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
//     i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
//     i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
//     i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)

	Data_6050[0] = 0x6A;
	Data_6050[1] = 0x00; //RESET
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	
	Data_6050[0] = 0x37;
	Data_6050[1] = 0x02; 
	if (MPU6050_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	
	for (t=0; t<50000; t++);
	
	Data_6050[0] = 0x00;
	Data_6050[1] = 0x70; 
	if (HMC5883_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	
	Data_6050[0] = 0x01;
	Data_6050[1] = 0xA0; 
	if (HMC5883_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	
	Data_6050[0] = 0x02;
	Data_6050[1] = 0x00; 
	if (HMC5883_ReadWrite(&Data_6050[0], 2, NULL, 0) == 0)
		return (0);
	
	for (t=0; t<50000; t++);

	return (1);
}

int8_t HMC5883_ReadAll(HMC5883_sensor *MAG)
{
	unsigned char TestData[6] = {0};
	unsigned char test = 3;
	int16_t Data[3] = {0};
	int i = 0;

	MAG->MAG_X = 0;
	MAG->MAG_Y = 0;
	MAG->MAG_Z = 0;

	if (HMC5883_ReadWrite(&test, 1, &TestData[0], 6) == 0)
		return (0);
	
	for (i=0; i<3; i++)
	{
		Data[i] = (TestData[(i*2)]<<8) | (TestData[(i*2)+1]);
	}
	
	MAG->MAG_X = Data[0];
	MAG->MAG_Y = Data[2];
	MAG->MAG_Z = Data[1];
	
	return (1);

}


int8_t GY87_init(void)
{
	int8_t sensor = 0;
	
	PINSEL_ConfigPin (0, 27, 1);
	PINSEL_ConfigPin (0, 28, 1);
	PINSEL_SetOpenDrainMode(0, 27, ENABLE);
	PINSEL_SetOpenDrainMode(0, 28, ENABLE);
	PINSEL_SetPinMode(0, 27, PINSEL_BASICMODE_PLAINOUT);
	PINSEL_SetPinMode(0, 28, PINSEL_BASICMODE_PLAINOUT);

	// Initialize Slave I2C peripheral
	I2C_Init((en_I2C_unitId)I2CDEV_M, 400000);
	/* Enable Slave I2C operation */
	
	I2C_Cmd((en_I2C_unitId)I2CDEV_M, I2C_MASTER_MODE, ENABLE);
	
	if (MPU6050_init() != 0)
		sensor = sensor | 0x01;
	else
		_DBG_("MPU6050 init ERROR");
	
	if (HMC5883_init() != 0)
	{
		sensor = sensor | 0x02;
	}
	else
		_DBG_("HMC5883 init ERROR");
	
	
	return (sensor);
}








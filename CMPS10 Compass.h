
#define CMPS10_address 0xC0>>1

// Register  		Function
// 0						Software version
// 1  					Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3  				Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
// 4 						Pitch angle - signed byte giving angle in degrees from the horizontal plane
// 5 						Roll angle - signed byte giving angle in degrees from the horizontal plane
// 6						Unused
// 7						Unused
// 8						Unused
// 9						Unused
// 10,11 				Magnetometer X axis raw output, 16 bit signed integer with register 10 being the upper 8 bits
// 12,13				Magnetometer Y axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15				Magnetometer Z axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
// 16,17				Accelerometer  X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
// 18,19				Accelerometer  Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21				Accelerometer  Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
// 22						Command register

typedef struct
{
	uint16_t 		Raw_MagX;
	uint16_t 		Raw_MagY;
	uint16_t 		Raw_MagZ;
	
	float    		AC_X;
	float    		AC_Y;
	float    		AC_Z;
	
	uint16_t    Compass; 
	int8_t 			Pitch;
	int8_t			Roll;
	void 				(*callback)(void);
} CPMS10_sensor;

int8_t CMPS10_ReadWrite(uint8_t* txdata, uint32_t txlen,uint8_t* rxdata, uint32_t rxlen);
int16_t CMPS10_RaadAll(CPMS10_sensor *sensor);


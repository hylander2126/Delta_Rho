//================================================================================================
//                                           L3GD20
//                             three-axis digital output gyroscope
//================================================================================================

#define L3GD20_address 0xD6

//________________________________________________________________________________________________
void L3GD20_init(void)
{
	char check;

	// CTRL_REG1 Initialization
	//      Normal mode operation
	//      All axes enabled
	//      Output data rate = 760 Hz
	//      Cut-Off = 100
	check = i2c_writebyte(L3GD20_address, 0x20, 0xFF);

	// CTRL_REG2 Initialization
	//      High-pass filter = Normal mode (reset reading HP_RESET_FILTER)
	//      High-pass filter cut off frequency = 0.09 Hz (@ODR = 760 Hz)
	check = i2c_writebyte(L3GD20_address, 0x21, 0x09);

	// CTRL_REG3 Initialization
	//      Interrupt on INT1 pin = disable
	//      Boot status on INT1 = disable
	//      Interrupt active configuration on INT1 = High
	//      Push-pull / Open drain = push-pull
	//      Date-ready on DRDY/INT2 = disable
	//      FIFO watermark interrupt on DRDY/INT2 = disable
	//      FIFO overrun interrupt on DRDY/INT2 Default value = disable
	//      FIFO empty interrupt on DRDY/INT2 = disable
	check = i2c_writebyte(L3GD20_address, 0x22, 0x00);

	// CTRL_REG4 Initialization
	//      Block data update = output registers not updated until MSb and LSb reading
	//      Big/little endian data selection = Data LSb @ lower address
	//      Full scale selection = 250 dps
	//      SPI serial interface mode selection = 4-wire interface
	check = i2c_writebyte(L3GD20_address, 0x23, 0x80);

}

//________________________________________________________________________________________________
unsigned char L3GD20_read_ID(void)
{
	unsigned char id;
	char check;
	check = i2c_readbyte(L3GD20_address, 0x0F, &id);
	if(check == 1) return id;
	else return 0;
}

//________________________________________________________________________________________________
unsigned char L3GD20_status(void)
{
	unsigned char status;
	char check;
	check = i2c_readbyte(L3GD20_address, 0x27, &status);
	if(check == 1) return status;
	else return -1;
}

//________________________________________________________________________________________________
char L3GD20_temp(void)
{
	char temp, check;
	check = i2c_readbyte(L3GD20_address, 0x26, &temp);
	if(check == 1){
		return temp;
	}
	else return -1;
}

//________________________________________________________________________________________________
char L3GD20_gyro(int *wx, int *wy, int *wz)
{
	char data_L, data_H, cL, cH, c;

	int TEMP;

	// Read angular velocity about x axis
	cL = i2c_readbyte(L3GD20_address, 0x28, &data_L);
	cH = i2c_readbyte(L3GD20_address, 0x29, &data_H);
	*wx = (data_H << 8) + data_L;
	c = cL + cH;

	// Read angular velocity about y axis
	cL = i2c_readbyte(L3GD20_address, 0x2A, &data_L);
	cH = i2c_readbyte(L3GD20_address, 0x2B, &data_H);
	*wy = (data_H << 8) + data_L;
	c = c + cL + cH;


	// Read angular velocity about z axis
	cL = i2c_readbyte(L3GD20_address, 0x2C, &data_L);
	cH = i2c_readbyte(L3GD20_address, 0x2D, &data_H);
	*wz = (data_H << 8) + data_L;
	c = c + cL + cH;

	if(c == 6) return 1;
	else return -1;
}
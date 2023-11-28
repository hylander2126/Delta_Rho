//================================================================================================
//                                        LSM303DLHC
//                     three-axis digital output Accelerometer and Compass
//================================================================================================

#define LSM303DLHC_acc_address 0x32
#define LSM303DLHC_mag_address 0x3C

//________________________________________________________________________________________________
void LSM303DLHC_init(void)
{
	char check;

	//------------------------------------------------------------------------------------------------
	//      ACCELEROMETER

	// CTRL_REG1_A Initialization
	//      Normal mode operation
	//      All axes enabled
	//      Output data rate = 400 Hz
	check = i2c_writebyte(LSM303DLHC_acc_address, 0x20, 0x77);

	// CTRL_REG2_A Initialization
	//      High-pass filter = Normal mode (reset reading HP_RESET_FILTER)
	//      Filtered data selection = internal filter bypassed
	//      High-pass filter cut off frequency =
	//      High pass filter enabled for CLICK function = filter bypassed
	//      High pass filter enabled for AOI function on Interrupt 2 = filter bypassed
	//      High pass filter enabled for AOI function on Interrupt 1 = filter bypassed
	check = i2c_writebyte(LSM303DLHC_acc_address, 0x21, 0x00);


	// CTRL_REG4_A Initialization
	//      Block data update = output registers not updated until MSb and LSb reading
	//      Big/little endian data selection = Data LSb @ lower address
	//      Full scale selection = +/- 4G
	//      High resolution output mode = enable
	//      SPI serial interface mode selection = 4-wire interface
	check = i2c_writebyte(LSM303DLHC_acc_address, 0x23, 0x98);



	//------------------------------------------------------------------------------------------------
	//      MAGNETOMETER

	// CRA_REG_M  Initialization
	//      Temperature sensor = enable
	//      Output data rate = 220 Hz
	check = i2c_writebyte(LSM303DLHC_mag_address, 0x00, 0x9C);

	// CRB_REG_M  Initialization
	//      Gain configuration bits = enable
	check = i2c_writebyte(LSM303DLHC_mag_address, 0x01, 0x20);

	// MR_REG_M Initialization
	//      Mode select = Continuous-conversion mode
	check = i2c_writebyte(LSM303DLHC_mag_address, 0x02, 0x00);
}

//________________________________________________________________________________________________
char LSM303DLHC_acc(int *ax, int *ay, int *az)
{
	char data_L, data_H, cL, cH, c;

	int TEMP;

	*ax = 0;
	*ay = 0;
	*az = 0;

	// Read angular velocity about x axis
	cL = i2c_readbyte(LSM303DLHC_acc_address, 0x28, &data_L);
	cH = i2c_readbyte(LSM303DLHC_acc_address, 0x29, &data_H);
	*ax = ((data_H << 8) + data_L) >> 3;
	c = cL + cH;

	// Read angular velocity about y axis
	cL = i2c_readbyte(LSM303DLHC_acc_address, 0x2A, &data_L);
	cH = i2c_readbyte(LSM303DLHC_acc_address, 0x2B, &data_H);
	*ay = ((data_H << 8) + data_L) >> 3;
	c = c + cL + cH;


	// Read angular velocity about z axis
	cL = i2c_readbyte(LSM303DLHC_acc_address, 0x2C, &data_L);
	cH = i2c_readbyte(LSM303DLHC_acc_address, 0x2D, &data_H);
	*az = ((data_H << 8) + data_L) >> 3;
	c = c + cL + cH;

	if(c == 6) return 1;
	else return -1;
}

//________________________________________________________________________________________________
char LSM303DLHC_mag(int *mx, int *my, int *mz)
{
	char data_L, data_H, cL, cH, c;

	int TEMP;

	*mx = 0;
	*my = 0;
	*mz = 0;

	// Read angular velocity about x axis
	cL = i2c_readbyte(LSM303DLHC_mag_address, 0x04, &data_L);
	cH = i2c_readbyte(LSM303DLHC_mag_address, 0x03, &data_H);
	*mx = (data_H << 8) + data_L;
	c = cL + cH;

	// Read angular velocity about y axis
	cL = i2c_readbyte(LSM303DLHC_mag_address, 0x08, &data_L);
	cH = i2c_readbyte(LSM303DLHC_mag_address, 0x07, &data_H);
	*my = (data_H << 8) + data_L;
	c = c + cL + cH;


	// Read angular velocity about z axis
	cL = i2c_readbyte(LSM303DLHC_mag_address, 0x06, &data_L);
	cH = i2c_readbyte(LSM303DLHC_mag_address, 0x05, &data_H);
	*mz = (data_H << 8) + data_L;
	c = c + cL + cH;

	if(c == 6) return 1;
	else return -1;
}

//________________________________________________________________________________________________
char LSM303DLHC_temp(int *temp)
{
	char data_L, data_H, cL, cH, c;

	int TEMP;

	*temp = 0;

	// Read the temperature
	cL = i2c_readbyte(LSM303DLHC_mag_address, 0x32, &data_L);
	cH = i2c_readbyte(LSM303DLHC_mag_address, 0x31, &data_H);
	*temp = (data_H << 8) + data_L;
	*temp = *temp >> 4;
	c = cL + cH;

	if(c == 2) return 1;
	else return -1;
}
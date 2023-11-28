//================================================================================================
//                                            I2C
//================================================================================================

#define MAX_TRIES 10
#define I2C_START 0
#define I2C_DATA  1
#define I2C_STOP  2


#define TW_START 0x08
#define TW_REP_START 0x10

#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
#define TW_MT_DATA_NACK 0x30
#define TW_MT_ARB_LOST 0x38

#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58
#define TW_MR_ARB_LOST 0x38


void twi_master_init(void){
	
	// SCL_frequency = CPU_clock_frequency / (16 + 2*TWBR*4^TWPS)

	TWSR = 0x00;   // Select Pre-scaler of 1
	TWBR = 0x04;
}

//________________________________________________________________________________________________
unsigned char i2c_transmit(unsigned char type) {
	switch(type) {
		case I2C_START:    // Send Start Condition
		TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		break;
		case I2C_DATA:     // Send Data
		TWCR = (1 << TWINT) | (1 << TWEN);
		break;
		case I2C_STOP:     // Send Stop Condition
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
		return 0;
	}
	// Wait for TWINT flag set in TWCR Register
	while (!(TWCR & (1 << TWINT)));
	// Return TWI Status Register, mask the Pre-scaler bits (TWPS1,TWPS0)
	return (TWSR & 0xF8);
}

//________________________________________________________________________________________________
int i2c_writebyte(unsigned char slave_address, unsigned char dev_addr,char data)
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;

	i2c_retry:
	if (n++ >= MAX_TRIES) return r_val;
	
	// Transmit Start Condition
	twi_status = i2c_transmit(I2C_START);
	// Check the TWI Status
	if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;
	
	
	// Send slave address (SLA_W)
	TWDR = slave_address & 0xFE;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;
	
	
	// Send the register Address
	TWDR = dev_addr;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;
	
	// Put data into data register and start transmission
	TWDR = data;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;
	// TWI Transmit Ok
	r_val=1;

	i2c_quit:

	// Transmit I2C Data
	twi_status = i2c_transmit(I2C_STOP);
	return r_val;
}

//________________________________________________________________________________________________
int i2c_readbyte(unsigned char slave_address, unsigned char dev_addr,char *data)
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	
	
	i2c_retry:
	if (n++ >= MAX_TRIES) return r_val;

	// Transmit Start Condition
	twi_status = i2c_transmit(I2C_START);
	// Check the TWSR status
	if (twi_status == TW_MR_ARB_LOST) goto i2c_retry;
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;

	// Send slave address (SLA_W)
	TWDR = slave_address & 0xFE;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;
	
	
	// Send the register Address
	TWDR = dev_addr;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;

	// Send start Condition
	twi_status=i2c_transmit(I2C_START);
	// Check the TWSR status
	if (twi_status == TW_MR_ARB_LOST) goto i2c_retry;
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;
	
	// Send slave address (SLA_W)
	TWDR = slave_address | 0x01;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if ((twi_status == TW_MR_SLA_NACK) || (twi_status == TW_MR_ARB_LOST)) goto i2c_retry;
	if (twi_status != TW_MR_SLA_ACK) goto i2c_quit;
	
	
	// Read I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	if (twi_status != TW_MR_DATA_NACK) goto i2c_quit;
	// Get the Data
	*data = TWDR;
	r_val=1;

	i2c_quit:

	// Send Stop Condition
	twi_status=i2c_transmit(I2C_STOP);
	return r_val;
}
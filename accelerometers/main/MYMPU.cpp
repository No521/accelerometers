/*
 * MYMPU.cpp
 *
 *  Created on: 24 jul. 2017
 *      Author: Enrique
 */

#include "MYMPU.h"
#include "MPU9250defines.h"
#include <esp_log.h>
#include <math.h>

static char tag1[] = "MPU_class";

MYMPU::MYMPU(gpio_num_t ar_sda_pin, gpio_num_t  ar_sclpin, uint8_t ar_i2cAddress, uint8_t ar_i2cAddressMag,
		i2c_port_t ar_i2c_num, i2c_mode_t ar_i2cmode, uint32_t ar_clkspeed)
: I2C(ar_sda_pin, ar_sclpin,ar_i2cAddress, ar_i2cAddressMag, ar_i2c_num, ar_i2cmode, ar_clkspeed) {
	/*
	cmd = NULL;
	i2c_num = ar_i2c_num;
	i2c_Address = ar_i2cAddress;
	i2c_AddressMag = ar_i2cAddressMag;
	conf.mode = ar_i2cmode;
	conf.sda_io_num =ar_sda_pin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num =  ar_sclpin;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = ar_clkspeed;
	 */
};

void MYMPU::getMres()
{
	switch (Mscale)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
		break;
	}
}

void MYMPU::getGres()
{
	switch (Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
	// 2-bit value:
	case GFS_250DPS:
		gRes = 250.0f / 32768.0f;
		break;
	case GFS_500DPS:
		gRes = 500.0f / 32768.0f;
		break;
	case GFS_1000DPS:
		gRes = 1000.0f / 32768.0f;
		break;
	case GFS_2000DPS:
		gRes = 2000.0f / 32768.0f;
		break;
	}
}

void MYMPU::getAres()
{
	switch (Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
	// 2-bit value:
	case AFS_2G:
		aRes = 2.0f / 32768.0f;
		break;
	case AFS_4G:
		aRes = 4.0f / 32768.0f;
		break;
	case AFS_8G:
		aRes = 8.0f / 32768.0f;
		break;
	case AFS_16G:
		aRes = 16.0f / 32768.0f;
		break;
	}
}


//void MYMPU::readAccelData(int16_t * destination)
void MYMPU::readAccelData(int16_t * destination)
{
	esp_err_t error = read_sensor( i2c_Address, ACCEL_XOUT_H, &rawData[0], 6);
	if(error != ESP_OK)
		printf("Lectura sensor acelerometro error %d\n", error);

	destination[0] = (rawData[0] << 8) | rawData[1];
	destination[1] = (rawData[2] << 8) | rawData[3];
	destination[2] = (rawData[4] << 8) | rawData[5];

	ESP_LOGD(tag1, "accel_x: %d, accel_y: %d, accel_z: %d",destination[0], destination[1], destination[2]);
	//printf(" Accel X=%d, Y=%d, Z=%d\n", destination[0], destination[1], destination[2]);
}

void MYMPU::readGyroData(int16_t * destination)
{
	// x/y/z gyro register data stored here  inside I2C class uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	esp_err_t error = read_sensor(i2c_Address, GYRO_XOUT_H, &rawData[6], (size_t) 6);
	if(error != ESP_OK)
		printf("Lectura sensor giroscopio error %d\n", error);

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[6] << 8) | rawData[7] ;
	destination[1] = ((int16_t)rawData[8] << 8) | rawData[9] ;
	destination[2] = ((int16_t)rawData[10] << 8) | rawData[11] ;
}

void MYMPU::readMagData(int16_t * destination)
{
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end
	// of data acquisition
	uint8_t rawData[7];
	esp_err_t error = read_sensor(	i2c_AddressMag, AK8963_ST1, &rawData[0], (size_t) 1);
	if(error != ESP_OK)
		printf("Lectura sensor  Brujula error %d\n", error);
	uint8_t c = rawData[0];
	// Wait for magnetometer data ready bit to be set
	if ( c & 0x01)
	{
		// Read the six raw data and ST2 registers sequentially into data array
		error = read_sensor(i2c_AddressMag, AK8963_XOUT_L, &rawData[0], (size_t) 7);
		if(error != ESP_OK)
			printf(" 2 Lectura sensor  Brujula %d\n", error);
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		// Check if magnetic sensor overflow set, if not then report data
		if (!(c & 0x08))
		{
			// Turn the MSB and LSB into a signed 16-bit value
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			// Data stored as little Endian
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
		else
		{
			printf("Brujula no lee %d\n", error);
		}
	}
}

int16_t MYMPU::readTempData(int16_t * destination)
{
	uint8_t rawData[2]; // x/y/z gyro register data stored here
	// Read the two raw data registers sequentially into data array
	esp_err_t error = read_sensor(i2c_Address, TEMP_OUT_H, &rawData[12], (size_t) 2);
	if(error != ESP_OK)
		printf("Lectura sensor temperatura %d\n", error);

	// Turn the MSB and LSB into a 16-bit value
	destination[0] =((int16_t)rawData[12] << 8) | rawData[13];

	return destination[0];
}

bool MYMPU::beginMYMPU(void){
	float desviacion_calibracion[6];
	i2c_begin();
	char c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	printf("Lectura de WHO AM I en MPU9250 0x%x debria ser 0x71\n", c);
	c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	printf("Lectura de WHO AM I en AK8963  0x%x debria ser 0x48\n", c);

	selfTestMYMPU(&desviacion_calibracion[0]);
	printf(" Desviacion Accel X=%f, Y=%f, Z=%f\n",
			desviacion_calibracion[0], desviacion_calibracion[1], desviacion_calibracion[2]);
	printf(" Desviacion Gyro  X=%f, Y=%f, Z=%f\n",
			desviacion_calibracion[3], desviacion_calibracion[4], desviacion_calibracion[5]);

	calibrateMYMPU(gyroBias, accelBias);
	initMYMPU();

	//
	initAK8963(&desviacion_calibracion[0]);
	printf(" Desviacion Magnetometer  X=%f, Y=%f, Z=%f\n",
			desviacion_calibracion[3], desviacion_calibracion[4], desviacion_calibracion[5]);
	// Get sensor resolutions, only need to do this once
	getAres();
	getGres();
	getMres();

	return 1;
}

void MYMPU::initMYMPU()
{
	uint8_t data[12];
	// wake up device
	// Clear sleep mode bit (6), enable all sensors
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	vTaskDelay(100/portTICK_PERIOD_MS); // Wait for all registers to reset

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	vTaskDelay(200/portTICK_PERIOD_MS);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
	// respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion
	// update rates cannot be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
	// 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above.
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
	// left-shifted into positions 4:3

	// get current GYRO_CONFIG register value
	esp_err_t error= read_sensor(i2c_Address, GYRO_CONFIG, &data[0], (size_t) 1);
	if(error != ESP_OK)
		printf("3 Lectura calibrate MPU %d\n", error);
	uint8_t c = data[0];
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
	// GYRO_CONFIG
	// c =| 0x00;
	// Write new GYRO_CONFIG value to register
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c );

	// Set accelerometer full-scale range configuration
	// Get current ACCEL_CONFIG register value
	error= read_sensor(i2c_Address, ACCEL_CONFIG, &data[0], (size_t) 1);
	if(error != ESP_OK)
		printf("3 Lectura calibrate MPU %d\n", error);
	//c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
	c = data[0];
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	// Write new ACCEL_CONFIG register value
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by
	// choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
	// 1.13 kHz
	// Get current ACCEL_CONFIG2 register value
	error= read_sensor(i2c_Address, ACCEL_CONFIG2, &data[0], (size_t) 1);
	if(error != ESP_OK)
		printf("3 Lectura calibrate MPU %d\n", error);
	c = data[0];
	//c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	// Write new ACCEL_CONFIG2 register value
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because
	// of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	// Enable data ready (bit 0) interrupt
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
	printf("Terminamos MPU init con error %d\n", error);
	vTaskDelay(100/portTICK_PERIOD_MS);;
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void MYMPU::calibrateMYMPU(float * gyroBias, float * accelBias)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	// Write a one to bit 7 reset bit; toggle reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, READ_FLAG);
	vTaskDelay(100/portTICK_PERIOD_MS);

	// get stable time source; Auto select clock source to be PLL gyroscope
	// reference if ready else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	vTaskDelay(200/portTICK_PERIOD_MS);

	// Configure device for bias calculation
	// Disable all interrupts
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
	// Disable FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
	// Turn on internal clock source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	// Disable I2C master
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
	// Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
	// Reset FIFO and DMP
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
	vTaskDelay(15/portTICK_PERIOD_MS);;

	// Configure MPU6050 gyro and accelerometer for bias calculation
	// Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x01);
	// Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	// Set accelerometer full-scale to 2 g, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
	// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
	// MPU-9150)
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
	vTaskDelay(40/portTICK_PERIOD_MS);  // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	// Disable gyro and accelerometer sensors for FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
	// Read FIFO sample count
	esp_err_t error= read_sensor(MPU9250_ADDRESS, FIFO_COUNTH, &data[0], (size_t) 2);
	if(error != ESP_OK)
		printf("3 Lectura calibrate MPU %d\n", error);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	// How many sets of full gyro and accelerometer data for averaging
	packet_count = fifo_count/12;
	printf("Numero de paquetes %d\n", packet_count);

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		// Read data for averaging
		//readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
		error= read_sensor(MPU9250_ADDRESS, FIFO_R_W, &data[0], (size_t) 12);
		if(error != ESP_OK)
			printf("3 Lectura calibrate MPU %d\n", error);
		// Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit
		// biases.
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}
	// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	if (accel_bias[2] > 0L)
	{
		accel_bias[2] -= (int32_t) accelsensitivity;
	}
	else
	{
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
	// format.
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
	// Biases are additive, so change sign on calculated average gyro biases
	data[1] = (-gyro_bias[0]/4)       & 0xFF;
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer
	// bias registers. These registers contain factory trim values which must be
	// added to the calculated accelerometer biases; on boot up these registers
	// will hold non-zero values. In addition, bit 0 of the lower byte must be
	// preserved since it is used for temperature compensation calculations.
	// Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	// A place to hold the factory accelerometer trim biases
	int32_t accel_bias_reg[3] = {0, 0, 0};
	// Read factory accelerometer trim values
	//esp_err_t error = read_sensor(i2c_Address, TEMP_OUT_H, &rawData[12], (size_t) 2);


	error = read_sensor(i2c_Address, XA_OFFSET_H, &data[0], (size_t) 2);
	if(error != ESP_OK)
		printf("Error en calibrate MPU %d\n", error);
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	//readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	error = read_sensor(i2c_Address, YA_OFFSET_H, &data[0], (size_t) 2);
	if(error != ESP_OK)
		printf("Error en calibrate MPU %d\n", error);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	//readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	error = read_sensor(i2c_Address, ZA_OFFSET_H, &data[0], (size_t) 2);
	if(error != ESP_OK)
		printf("Error en calibrate MPU %d\n", error);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	// Define mask for temperature compensation bit 0 of lower byte of
	// accelerometer bias registers
	uint32_t mask = 1uL;
	// Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = {0, 0, 0};

	for (ii = 0; ii < 3; ii++)
	{
		// If temperature compensation bit is set, record that fact in mask_bit
		if ((accel_bias_reg[ii] & mask))
		{
			mask_bit[ii] = 0x01;
		}
	}

	// Construct total accelerometer bias, including calculated average
	// accelerometer bias from above
	// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
	// (16 g full scale)
	accel_bias_reg[0] -= (accel_bias[0]/8);
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	// preserve temperature compensation bit when writing back to accelerometer
	// bias registers
	data[1] = data[1] | mask_bit[0];
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	// Preserve temperature compensation bit when writing back to accelerometer
	// bias registers
	data[3] = data[3] | mask_bit[1];
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	// Preserve temperature compensation bit when writing back to accelerometer
	// bias registers
	data[5] = data[5] | mask_bit[2];

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
	accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
	accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
	printf("Terminamos MPU calibrate con error %d\n", error);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation matrix of 6 floats from factory trim values, +/- 14 or less
// deviation is a pass.
void MYMPU::selfTestMYMPU(float * destination)
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	// Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
	// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x02);
	// Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);
	// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);
	// Set full scale range for the accelerometer to 2 g
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS);

	// Get average current values of gyro and acclerometer
	for (int ii = 0; ii < 200; ii++)
	{
		//Serial.print("BHW::ii = ");
		//Serial.println(ii);
		// Read the six raw data registers into data array
		error = read_sensor(MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6);
		//readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
		// Turn the MSB and LSB into a signed 16-bit value
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		// Read the six raw data registers sequentially into data array
		// readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
		error = read_sensor(MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);
		// Turn the MSB and LSB into a signed 16-bit value
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	// Get average of 200 values and store as average current readings
	for (int ii =0; ii < 3; ii++)
	{
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	// Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
	// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
	vTaskDelay(25/portTICK_PERIOD_MS);  // Delay a while to let the device stabilize

	// Get average self-test values of gyro and accelerometer
	for (int ii = 0; ii < 200; ii++)
	{
		// Read the six raw data registers into data array
		// readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
		error = read_sensor(MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6);
		// Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		// Read the six raw data registers sequentially into data array
		// readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
		error = read_sensor(MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);
		// Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	// Get average of 200 values and store as average self-test readings
	for (int ii =0; ii < 3; ii++)
	{
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	vTaskDelay(25/portTICK_PERIOD_MS);   // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	// X-axis accel self-test results
	selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);
	// Y-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
	// Z-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);
	// X-axis gyro self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);
	// Y-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);
	// Z-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);

	// Retrieve factory self-test value from self-test code reads
	// FT[Xa] factory trim calculation
	factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
	// FT[Ya] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
	// FT[Za] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
	// FT[Xg] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
	// FT[Yg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
	// FT[Zg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
	// of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++)	{
		// Report percent differences
		destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;
		// Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.;
	}
}


// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void MYMPU::magCalMYMPU(float * bias_dest, float * scale_dest)
{
	printf("Mag Calibration!\n");
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3]  = {0, 0, 0},
			mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3]  = {(int16_t)0x8000, (int16_t)0x8000, (int16_t)0x8000},
			mag_min[3]  = {(int16_t)0x7FFF, (int16_t)0x7FFF,(int16_t)0x7FFF},
			mag_temp[3] = {0, 0, 0};

	// Make sure resolution has been calculated
	getMres();

	//Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
	/*Serial.println(
      F("  4 seconds to get ready followed by 15 seconds of sampling)"));
	 */
	vTaskDelay(4000/portTICK_PERIOD_MS);;

	// shoot for ~fifteen seconds of mag data
	// at 8 Hz ODR, new mag data is available every 125 ms
	if (Mmode == M_8HZ)
	{
		sample_count = 128;
	}
	// at 100 Hz ODR, new mag data is available every 10 ms
	if (Mmode == M_100HZ)
	{
		sample_count = 1500;
	}

	for (ii = 0; ii < sample_count; ii++)
	{
		readMagData(mag_temp);  // Read the mag data

		for (int jj = 0; jj < 3; jj++)
		{
			if (mag_temp[jj] > mag_max[jj])
			{
				mag_max[jj] = mag_temp[jj];
			}
			if (mag_temp[jj] < mag_min[jj])
			{
				mag_min[jj] = mag_temp[jj];
			}
		}

		if (Mmode == M_8HZ)
		{
			vTaskDelay(135/portTICK_PERIOD_MS);  // At 8 Hz ODR, new mag data is available every 125 ms
		}
		if (Mmode == M_100HZ)
		{
			vTaskDelay(12/portTICK_PERIOD_MS);   // At 100 Hz ODR, new mag data is available every 10 ms
		}
	}

	// Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	// Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	// Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	// Get 'average' x mag bias in counts
	mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
	// Get 'average' y mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
	// Get 'average' z mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

	// Save mag biases in G for main program
	bias_dest[0] = (float)mag_bias[0] * mRes * factoryMagCalibration[0];
	bias_dest[1] = (float)mag_bias[1] * mRes * factoryMagCalibration[1];
	bias_dest[2] = (float)mag_bias[2] * mRes * factoryMagCalibration[2];

	// Get soft iron correction estimate
	// Get average x axis max chord length in counts
	mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
	// Get average y axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
	// Get average z axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	scale_dest[0] = avg_rad / ((float)mag_scale[0]);
	scale_dest[1] = avg_rad / ((float)mag_scale[1]);
	scale_dest[2] = avg_rad / ((float)mag_scale[2]);

	printf("Mag Calibration done!\n");
}


void MYMPU::initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	// TODO: Test this!! Likely doesn't work
	error = writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	if(error != ESP_OK)
		printf("Error en Power down initAK8963 %d\n", error);
	vTaskDelay(500/portTICK_PERIOD_MS);  // solo hace falta 10 ms
	error = writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	if(error != ESP_OK)
		printf("Error en fuse initAK8963 %d\n", error);
	vTaskDelay(500/portTICK_PERIOD_MS);

	// Read the x-, y-, and z-axis calibration values
	error = read_sensor(AK8963_ADDRESS, AK8963_ASAX, &rawData[0],3);
	if(error != ESP_OK)
		printf("Error en fuse initAK8963 %d\n", error);
	// Return x-axis sensitivity adjustment values, etc.
	destination[0] =  (float)(rawData[0] - 128)/256. + 1.; // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
	destination[2] =  (float)(rawData[2] - 128)/256. + 1.;

	error = writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	if(error != ESP_OK)
		printf("Error en fuse initAK8963 %d\n", error);
	vTaskDelay(500/portTICK_PERIOD_MS);

	// Configure the magnetometer for continuous read and highest resolution.
	// Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	// register, and enable continuous mode data acquisition Mmode (bits [3:0]),
	// 0010 for 8 Hz and 0110 for 100 Hz sample rates.

	// Set magnetometer data resolution and sample ODR
	error = writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
	if(error != ESP_OK)
		printf("Error en fuse initAK8963 %d\n", error);
	vTaskDelay(500/portTICK_PERIOD_MS);
}

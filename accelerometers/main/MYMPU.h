/*
 * MYMPU.h
 *
 *  Created on: 24 jul. 2017
 *      Author: Enrique
 */

#ifndef MAIN_MYMPU_H_
#define MAIN_MYMPU_H_

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <cstdint>

#include "I2Cmio.h"
#include "sdkconfig.h"


class MYMPU : public I2C
{
protected:

	// Set initial input parameters
	enum Ascale
	{
		AFS_2G = 0,
		AFS_4G,
		AFS_8G,
		AFS_16G
	};

	enum Gscale {
		GFS_250DPS = 0,
		GFS_500DPS,
		GFS_1000DPS,
		GFS_2000DPS
	};

	enum Mscale {
		MFS_14BITS = 0, // 0.6 mG per LSB
		MFS_16BITS      // 0.15 mG per LSB
	};

	enum M_MODE {
		M_8HZ = 0x02,  // 8 Hz update
		M_100HZ = 0x06 // 100 Hz continuous magnetometer
	};

	// TODO: Add setter methods for this hard coded stuff
	// Specify sensor full scale
	uint8_t Gscale = GFS_250DPS;
	uint8_t Ascale = AFS_2G;
	// Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t Mscale = MFS_16BITS;

	// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	uint8_t Mmode = M_8HZ;


	bool magInit();
	void kickHardware();


public:
	float pitch = 0, yaw = 0, roll = 0;
	float temperature = 0;   				// Stores the real internal chip temperature in Celsius

	uint32_t delt_t = 0; 					// Used to control display output rate

	uint32_t count = 0, sumCount = 0; 		// used to control display output rate
	float deltat = 0.0f, sum = 0.0f;  		// integration interval for both filter schemes
	uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
	uint32_t Now = 0;        				// used to calculate integration interval

	// Stores the 16-bit signed accelerometer sensor output
	int16_t accelCount[3] = {0, 0, 0};   	// Stores the 16-bit signed count sensor output
	int16_t gyroCount[3] = {0, 0, 0};   	// Stores the 16-bit signed gyro sensor output
	int16_t magCount[3] = {0, 0, 0};    	// Stores the 16-bit signed magnetometer sensor output
	int16_t tempCount[1]= { 0};   				// Temperature raw count output

	// Scale resolutions per LSB for the sensors
	float aRes, gRes, mRes;
	// Variables to hold latest sensor data values
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	// Factory mag calibration and mag bias
	float factoryMagCalibration[3] = {0, 0, 0}, factoryMagBias[3] = {0, 0, 0};
	// Bias corrections for gyro, accelerometer, and magnetometer
	float 	gyroBias[3]  = {0, 0, 0},
			accelBias[3] = {0, 0, 0},
			magBias[3]   = {0, 0, 0},
			magScale[3]  = {0, 0, 0};
	float selfTest[6];



	// Public method declarations
	MYMPU(gpio_num_t ar_sda_pin, gpio_num_t  ar_sclpin, uint8_t ar_i2cAddress, uint8_t ar_i2cAddressMag,
			i2c_port_t ar_i2c_num, i2c_mode_t ar_i2cmode, uint32_t ar_clkspeed);
	void initMYMPU();
	bool beginMYMPU(void);
	void calibrateMYMPU(float * gyroBias, float * accelBias);
	void selfTestMYMPU(float * destination);
	void initAK8963(float * destination);
	void magCalMYMPU(float * bias_dest, float * scale_dest);
	void getMres();
	void getGres();
	void getAres();
	void readAccelData(int16_t *);
	void readGyroData(int16_t *);
	void readMagData(int16_t *);
	int16_t readTempData(int16_t *);
	void updateTime();
};  // class MYMPU

#endif /* MAIN_MYMPU_H_ */

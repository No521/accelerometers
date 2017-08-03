/*
 * Fichero principal
 *
 */


#include "I2C.cpp"
#include "I2Cmio.h"
#include "MPU9250defines.h"
#include "MYMPU.h"

extern "C" {
void app_main(void );
}


void task_readAcceleration(void *pvparameters) {

	int16_t accel[3];
	MYMPU* myIMU = (MYMPU *)pvparameters;
	while(1){

		myIMU->readAccelData( myIMU->accelCount);
		printf(" Accel X=%d, Y=%d, Z=%d\n", myIMU->accelCount[0], myIMU->accelCount[1], myIMU->accelCount[2]);
		myIMU->readGyroData( myIMU->gyroCount );
		printf(" Gyro  X=%d, Y=%d, Z=%d", myIMU->gyroCount [0], myIMU->gyroCount [1], myIMU->gyroCount [2]);
		myIMU->readTempData( myIMU->tempCount  );
		printf("    Temp =%f ºC\n", (float)myIMU->tempCount[0]/ 333.87 +21.0);
		myIMU->readMagData( myIMU->magCount );
		printf(" Mag   X=%d, Y=%d, Z=%d\n", myIMU->magCount [0], myIMU->magCount [1], myIMU->magCount [2]);


		// If intPin goes high, all data registers have new data
		// On interrupt, check if data ready interrupt
		if ((myIMU->readByte(MPU9250_ADDRESS, INT_STATUS)) & 0x01)
		{
	 		myIMU->readAccelData(myIMU->accelCount);  // Read the x/y/z adc values
			// Now we'll calculate the accleration value into actual g's
			// This depends on scale being set
			myIMU->ax = (float)myIMU->accelCount[0] * myIMU->aRes; // - myIMU.accelBias[0];
			myIMU->ay = (float)myIMU->accelCount[1] * myIMU->aRes; // - myIMU.accelBias[1];
			myIMU->az = (float)myIMU->accelCount[2] * myIMU->aRes; // - myIMU.accelBias[2];
			printf(" Accel X=%f, Y=%f, Z=%f mg\n", myIMU->ax, myIMU->ay, myIMU->az);
			myIMU->readGyroData(myIMU->gyroCount);  // Read the x/y/z adc values
			// Calculate the gyro value into actual degrees per second
			// This depends on scale being set
			myIMU->gx = (float)myIMU->gyroCount[0] * myIMU->gRes;
			myIMU->gy = (float)myIMU->gyroCount[1] * myIMU->gRes;
			myIMU->gz = (float)myIMU->gyroCount[2] * myIMU->gRes;

			myIMU->readMagData(myIMU->magCount);  // Read the x/y/z adc values
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental
			// corrections
			// Get actual magnetometer value, this depends on scale being set
			myIMU->mx = (float)myIMU->magCount[0] * myIMU->mRes
					* myIMU->factoryMagCalibration[0] - myIMU->magBias[0];
			myIMU->my = (float)myIMU->magCount[1] * myIMU->mRes
					* myIMU->factoryMagCalibration[1] - myIMU->magBias[1];
			myIMU->mz = (float)myIMU->magCount[2] * myIMU->mRes
					* myIMU->factoryMagCalibration[2] - myIMU->magBias[2];
		} // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
		vTaskDelay(500 / portTICK_PERIOD_MS);
	} // fin del loop while(1)

	vTaskDelete(NULL);
}


void app_main()
{
	MYMPU *MPU;
	float desviacion_calibracion[6];
	MPU = new MYMPU((gpio_num_t)SDA_PIN, (gpio_num_t)SCL_PIN, (uint8_t)MPU9250_ADDRESS, (uint8_t)AK8963_ADDRESS,
			(i2c_port_t)I2C_NUM_0, (i2c_mode_t)I2C_MODE_MASTER, (uint32_t)I2C_EXAMPLE_MASTER_FREQ_HZ );
	MPU->beginMYMPU();
	MPU->i2cScannig();

	printf("Factory   2\n");
	vTaskDelay(100 / portTICK_PERIOD_MS);
	/*
	MPU->initAK8963(&desviacion_calibracion[0]);
	printf(" Desviacion Magnetometer  X=%f, Y=%f, Z=%f\n", desviacion_calibracion[3], desviacion_calibracion[4], desviacion_calibracion[5]);
	MPU->magCalMYMPU(MPU->magBias, MPU->magScale);
	 */
	xTaskCreate(&task_readAcceleration, "Read Acceleration", 2048, MPU, 4, NULL);
}

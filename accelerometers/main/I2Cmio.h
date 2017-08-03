/*
 * I2Cmio.h
 *
 *  Created on: 22 jul. 2017
 *      Author: Enrique
 */

#ifndef MAIN_I2CMIO_H_
#define MAIN_I2CMIO_H_

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <stddef.h>
#include <cstdint>

#include "sdkconfig.h"


#define SDA_PIN 18
#define SCL_PIN 19


#define I2C_ADDRESS 0x68 // I2C address of MPU6050
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define I2C_EXAMPLE_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

/*
typedef  unsigned char uint8_t;
typedef  signed char int8_t;
typedef  unsigned short uint16_t;
typedef  signed short int16_t;
typedef  unsigned int uint32_t;
typedef  int int32_t;
*/
class I2C
{
protected:
	i2c_port_t i2c_num;
	uint8_t i2c_Address;
	uint8_t i2c_AddressMag;
	i2c_cmd_handle_t cmd;
	i2c_config_t conf;
	uint8_t rawData[21];  // para todos los datos 6 por accel gyro y 7 para mag, incluye st2 y 2 para temp
	esp_err_t error = ESP_OK;

public:
	I2C( gpio_num_t ar_sda_pin, gpio_num_t  ar_sclpin, uint8_t ar_i2cAddress, uint8_t ar_i2cAddressMag,
					i2c_port_t ar_i2c_num, i2c_mode_t ar_i2cmode, uint32_t ar_clkspeed);
	void i2c_begin(void);
	esp_err_t read_slave(uint8_t ar_i2c_Address, uint8_t* data_rd, size_t size);
	esp_err_t write_slave(uint8_t ar_i2c_Address, uint8_t* data_wr, size_t size);
	esp_err_t read_sensor(uint8_t ar_i2c_Address, uint8_t reg_entrada_lectura, uint8_t* data_rd, size_t size);
	esp_err_t writeByte(uint8_t ar_i2c_Address, uint8_t reg_entrada_escritura, uint8_t data_wr);
	uint8_t	  readByte(uint8_t ar_i2c_Address, uint8_t reg_a_leer);
	esp_err_t writeByteS(uint8_t ar_i2c_Address, uint8_t reg_entrada_escritura, uint8_t* data_rd, size_t size );
	esp_err_t i2cScannig(void);
};



#endif /* MAIN_I2CMIO_H_ */

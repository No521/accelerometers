/*
 * I2C.cpp
 *
 *  Created on: 22 jul. 2017
 *      Author: Enrique
 */
#include <esp_log.h>
#include "I2Cmio.h"
#include "sdkconfig.h"

static char tag1[] = "i2c_class";

I2C::I2C( gpio_num_t ar_sda_pin, gpio_num_t  ar_sclpin, uint8_t ar_i2cAddress, uint8_t ar_i2cAddressMag,
		i2c_port_t ar_i2c_num, i2c_mode_t ar_i2cmode, uint32_t ar_clkspeed)
{
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
}

void I2C::i2c_begin()  // con mayusculas I2C funcion mia
{
	ESP_LOGD(tag1, ">> mpu6050");
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	esp_err_t error = i2c_driver_install(I2C_NUM_0, conf.mode,
			I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
			I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
	if(error != ESP_OK)
		printf("Error de instalacion %d\n", error);
	vTaskDelay(200/portTICK_PERIOD_MS); // ajuste de timming
}


/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */

esp_err_t I2C::read_slave(uint8_t ar_i2c_Address, uint8_t* data_rd, size_t size)
{
	if (size == 0) {
		return ESP_OK;
	}
	// tiene que ya estar escrito el comando del registro a leer
	// lectura de los registros
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( ar_i2c_Address<< 1 ) | I2C_MASTER_READ, ACK_CHECK_EN));
	if (size > 1) {
		ESP_ERROR_CHECK(i2c_master_read(cmd, data_rd, size - 1, ACK_VAL));
	}
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	esp_err_t error = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if(error != ESP_OK)
		printf("Lectura registro Error %d\n", error);

	return error;
}
esp_err_t I2C::writeByte(uint8_t ar_i2c_Address,uint8_t reg_entrada_escritura, uint8_t data_wr)
{
	unsigned char  datos[2];
	datos[0] = reg_entrada_escritura;
	datos[1] = data_wr;
	size_t longitud = 2;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( ar_i2c_Address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write(cmd, &datos[0], longitud, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	esp_err_t error = i2c_master_cmd_begin(i2c_num, cmd, 1000/ portTICK_RATE_MS); // estaba a 1000 lo he cambiado a 20 y funciona, pero creo que es un limite
	i2c_cmd_link_delete(cmd);
	if(error != ESP_OK)
		printf("Escritura registro Error %d\n", error);
	return error;
}


/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t I2C::write_slave(uint8_t ar_i2c_Address, uint8_t* data_wr, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( ar_i2c_Address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	error = i2c_master_cmd_begin(i2c_num, cmd, 1000/ portTICK_RATE_MS); // estaba a 1000 lo he cambiado a 20 y funciona, pero creo que es un limite
	i2c_cmd_link_delete(cmd);
	if(error != ESP_OK)
		printf("Escritura registro Error %d\n", error);

	return error;
}

esp_err_t I2C::read_sensor(uint8_t ar_i2c_Address, uint8_t reg_entrada_lectura, uint8_t* data_rd, size_t size)
{
	if (size == 0) {
		return ESP_OK;
	}

	uint8_t registro[1];
	registro[0] = reg_entrada_lectura;

	// Posiciona the MPU6050 to position the internal register pointer to register ejempo MPU6050_ACCEL_XOUT_H.
	error = write_slave(ar_i2c_Address, registro, (size_t) 1);
	if(error != ESP_OK)
		printf("Read sensor escritura error %d\n", error);

	// lectura de los registros
	error = read_slave(ar_i2c_Address, data_rd, size);
	if(error != ESP_OK)
		printf("Read sensor Lectura registro error %d\n", error);

	return error;
}

esp_err_t I2C::writeByteS(uint8_t ar_i2c_Address, uint8_t reg_entrada_escritura, uint8_t* data_rd, size_t size )
{
	uint8_t registro[1];
	registro[0] = reg_entrada_escritura;

	// primero se escribe el registro donde se va a escribir
	error =  write_slave(ar_i2c_Address,&registro[0],1);
	if(error != ESP_OK)
		printf("Error en escritura del registro a donde escribir %d\n", error);
	// luego se escriben el registro los datos
	error =  write_slave(ar_i2c_Address,data_rd,size);
	if(error != ESP_OK)
		printf("Error en escritura del registro a donde escribir %d\n", error);

	return error;
}
uint8_t	 I2C::readByte(uint8_t ar_i2c_Address, uint8_t reg_a_leer)
{
	uint8_t registro[1];
	registro[0] = reg_a_leer;

	// Posiciona the MPU6050 to position the internal register pointer to register ejempo MPU6050_ACCEL_XOUT_H.
	error = write_slave(ar_i2c_Address, registro, (size_t) 1);
	if(error != ESP_OK)
		printf("Read sensor escritura error %d\n", error);
	// lectura de los registros
	error = read_slave(ar_i2c_Address, &registro[0], 1);
	if(error != ESP_OK)
		printf("Read sensor Lectura registro error %d\n", error);
	return registro[0];

}

esp_err_t I2C::i2cScannig(void){

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	return espRc;
}

/*
 * Sensor.c
 *
 * Created: 10-06-2017 08:44:56
 * Author : Mareks Bartasevics
 */ 

#define F_CPU 16000000UL

#define MPU_ADDRESS 0x68
#define MPU6050_GETATTITUDE  0


#define light_pwm_pin	PIND6
#define inside_temp_channel		1
#define outside_temp_channel	2
#define outside_light_channel	3

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_GAIN 16384.0



#include <avr/io.h>
#include <stdio.h>


#include "hc06.h"
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "mpu6050.h"

void dataPrint();
double readData(uint8_t highByte, uint8_t lowByte);

void adc_init(void);
uint16_t adc_read(uint8_t channel);

void pwm_init(void);

void spi_slave_init(void);
uint8_t spi_transieve(uint8_t value);

uint8_t process_temp (uint8_t adc_channel); 

double axg, ayg, azg, temp;
uint8_t tempOutside, lightOutside, tempInside;
uint8_t numReadings = 128;

 int main(void)
 {
	 usart_init();
	 i2c_init();
	mpu6050_init();

	spi_slave_init();
	pwm_init();
	adc_init();
	

while(1)
{
	axg = readData(MPU6050_RA_ACCEL_XOUT_H, MPU6050_RA_ACCEL_XOUT_L)/MPU6050_GAIN*100;
	ayg = readData(MPU6050_RA_ACCEL_YOUT_H, MPU6050_RA_ACCEL_YOUT_L)/MPU6050_GAIN*100;
	azg = readData(MPU6050_RA_ACCEL_ZOUT_H, MPU6050_RA_ACCEL_ZOUT_L)/MPU6050_GAIN*100;
	temp = readData(MPU6050_RA_TEMP_OUT_H, MPU6050_RA_TEMP_OUT_L);
	tempOutside = process_temp(outside_temp_channel);
	tempInside = process_temp(inside_temp_channel);
	lightOutside = adc_read(outside_light_channel);

	dataPrint();
	spi_transieve(tempOutside);
}


 }


void dataPrint()
{
	
	char bufferAXG[10];
	char bufferAYG[10];
	char bufferAZG[10];
	
	char bufferTEMP[10];
	
	itoa(axg, bufferAXG, 10);
	itoa(ayg, bufferAYG, 10);
	itoa(azg, bufferAZG, 10);
	
	itoa(temp, bufferTEMP,10);
	
	hc_06_bluetooth_transmit_string("Accel X: ");
	hc_06_bluetooth_transmit_string(bufferAXG);
	hc_06_bluetooth_transmit_string("  Accel Y: ");
	hc_06_bluetooth_transmit_string(bufferAYG);
	hc_06_bluetooth_transmit_string("  Accel Z: ");
	hc_06_bluetooth_transmit_string(bufferAZG);

	
	hc_06_bluetooth_transmit_string("  Temp: ");
	hc_06_bluetooth_transmit_string(bufferTEMP);
	
	hc_06_bluetooth_transmit_byte(0x0a);
	_delay_ms(8);
	
}



double readData(uint8_t highByte, uint8_t lowByte)
{
	i2c_start(MPU6050_ADDR | I2C_WRITE);
	i2c_write(highByte);
	_delay_us(10);
	//read data
	i2c_start(MPU6050_ADDR | I2C_READ);
	uint8_t data = i2c_readNak();
	i2c_stop();
	
	i2c_start(MPU6050_ADDR | I2C_WRITE);
	i2c_write(lowByte);
	_delay_us(10);
	//read data
	i2c_start(MPU6050_ADDR | I2C_READ);
	uint8_t data2 = i2c_readNak();
	i2c_stop();
	
	double value = (data<<8) | data2;

	return value;
	
	
}

void adc_init()
{
	// Select Vref=AVcc
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

uint16_t adc_read(uint8_t channel)
{
	//Clear previously read channel
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	
	//Start new conversion, singe conversion mode
	ADCSRA |= (1<< ADSC);
	
	//Wait until the conversion is completed
	while (ADCSRA & (1<<ADSC));
	
	return ADC;
}

void pwm_init()
{
	DDRD |= (1 << light_pwm_pin); // PIND6 is and output
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); // set none-inverting mode and fast PWM
	TCCR0B |= (0<<CS02) | (1 << CS01) | (1<<CS00);// set prescaler to 64 and starts PWM
	TCNT0 = 0; 	//reset TCNT
	OCR0A = 0; //Rest counter
}

void spi_slave_init()
{
	//Set MISO as output
	DDRB |= (1<<PINB4);
	
	//Enable SPI & Interupts
	SPCR |= (1<<SPE);
}

uint8_t spi_transieve(uint8_t value)
{
	//Load value to send
	SPDR = value;
	
	//wait for the SPI bus to finish
	
	while(!(SPSR & (1<<SPIF)));
	
	//Return received value
	return SPDR;
}

uint8_t process_temp(uint8_t adc_channel)
{
	int single_temp;
	unsigned int temp;
	unsigned int temp_total;

	temp_total = 0;

	for (int x = 0; x<numReadings;x++)
	{
		temp_total += single_temp = (5*adc_read(adc_channel)*100)/1024;
	}
	
	temp = temp_total/numReadings;

	return temp;
}
/*
 * MSP430 Library for the ADXL362 accelerometer
 *  by Vitor Kieling
 */

#ifndef ADXL362_H_
#define ADXL362_H_

/************************************
 *
 * SPI defines and pins
 *
 ************************************/
#define SPI_CLK 			8000000  	// 8 Mhz

#define SPI_BASE           	EUSCI_B0_BASE

#define SPI_CLK_PORT       	GPIO_PORT_P1
#define SPI_CLK_PIN        	GPIO_PIN5

#define SPI_COMM_PORT      	GPIO_PORT_P4
#define SPI_TX_PIN         	GPIO_PIN3
#define SPI_RX_PIN         	GPIO_PIN2

#define SPI_CS_PORT       	GPIO_PORT_P1
#define SPI_CS_PIN        	GPIO_PIN3

// Interrupt ports
#define SPI_INT1_PORT    	GPIO_PORT_P3
#define SPI_INT1_PIN     	GPIO_PIN7
#define SPI_INT2_PORT  		GPIO_PORT_P3
#define SPI_INT2_PIN    	GPIO_PIN6

#define SPI_SELECT_FUNCTION 	GPIO_PRIMARY_MODULE_FUNCTION

/************************************
 *
 * ADXL362 Register mappings
 *
 ************************************/
// Read only
#define REG_XDATA 0x08
#define REG_YDATA 0x09
#define REG_ZDATA 0x0A

#define REG_STATUS 0x0B

// N. of entries on FIFO
#define FIFO_ENTRIES_L 0x0C
#define FIFO_ENTRIES_H 0X0D

// Temp sensor data
#define TEMP_L 0x14
#define TEMP_H 0x15

// Write only
// write 0x52 to this to reset the adxl362, 0.5ms wait is required after reset
#define SOFT_RESET 0x1F

// Read-Write
#define THRESH_ACT_L 0x20
#define THRESH_ACT_H 0x21
#define THRESH_INACT_L 0x23
#define THRESH_INACT_H 0x24

#define TIME_ACT 0x22
#define TIME_INACT_L 0x25
#define TIME_INACT_H 0x26

#define ACT_INACT_CTL 0x27

#define FIFO_CTL 0x28
#define FIFO_SAMPLES 0x29

#define INTMAP1 0x2A
#define INTMAP2 0x2B

#define FILTER_CTL 0x2C
#define POWER_CTL 0x2D

/************************************
 *
 * Function definitions
 *
 ************************************/

void ADXL362_initSPI();
void ADXL362_initGPIO();

void ADXL362_sendSPI(uint8_t TXData);
uint8_t ADXL362_rcvSPI();

void ADXL362_writeReg(uint8_t reg, uint8_t TXData);
uint8_t ADXL362_readReg(uint8_t reg);

uint16_t ADXL362_readFIFO();

void ADXL362_beginMeasure();

void ADXL362_emptyFIFO();

int8_t processFIFO();

#endif /* ADXL362_H_ */

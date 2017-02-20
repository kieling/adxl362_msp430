#include <driverlib.h>
#include "adxl362.h"

void ADXL362_initSPI(){
    //Initialize Master
    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource 	= EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency 	= CS_getSMCLK();
    param.desiredSpiClock 		= SPI_CLK;
    param.msbFirst 		  		= EUSCI_B_SPI_MSB_FIRST;
    param.clockPhase 			= EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity 		= EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode 				= EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW;

    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &param);

    // Set CS pin func
    EUSCI_B_SPI_select4PinFunctionality(EUSCI_B0_BASE, EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);

    //Disable SPI module
    EUSCI_B_SPI_disable(EUSCI_B0_BASE);

    //Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

//    //Clear receive interrupt flag
//    EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
//
//    // Enable USCI_B0 RX interrupt
//    EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);

    //Wait for slave to initialize
    __delay_cycles(100);
}

// Pins used for SPI communication
void ADXL362_initGPIO(){
	// CLK pin
    GPIO_setAsPeripheralModuleFunctionInputPin(
			SPI_CLK_PORT,
			SPI_CLK_PIN,
			SPI_SELECT_FUNCTION
			);

    // TX pin
    GPIO_setAsPeripheralModuleFunctionOutputPin(
			SPI_COMM_PORT,
			SPI_TX_PIN,
			SPI_SELECT_FUNCTION
			);

    // RX pin
    GPIO_setAsPeripheralModuleFunctionInputPin(
			SPI_COMM_PORT,
			SPI_RX_PIN,
			SPI_SELECT_FUNCTION
			);

    // CS pin
    GPIO_setAsOutputPin(SPI_CS_PORT, SPI_CS_PIN);
    GPIO_setOutputHighOnPin(SPI_CS_PORT,SPI_CS_PIN);

    // INT1 pin
    GPIO_selectInterruptEdge(SPI_INT1_PORT, SPI_INT1_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(SPI_INT1_PORT, SPI_INT1_PIN);
    GPIO_clearInterrupt(SPI_INT1_PORT, SPI_INT1_PIN);
    GPIO_enableInterrupt(SPI_INT1_PORT, SPI_INT1_PIN);

//    // INT2 pin
//    GPIO_selectInterruptEdge(SPI_INT2_PORT, SPI_INT2_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
//    GPIO_setAsInputPinWithPullUpResistor(SPI_INT2_PORT, SPI_INT2_PIN);
//    GPIO_clearInterrupt(SPI_INT2_PORT, SPI_INT2_PIN);
//    GPIO_enableInterrupt(SPI_INT2_PORT, SPI_INT2_PIN);
}

void ADXL362_sendSPI(uint8_t TXData){
	// Wait buffer ready
    while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));
    // Transmit to slave
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, TXData);
}

uint8_t ADXL362_rcvSPI(){
    while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, 0x00); // Send dummy to CLK read

    //If RCV FLAG SET ; read data
    while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT));

    // Send to slave
    return EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
}

void ADXL362_writeReg(uint8_t reg, uint8_t TXData){
    while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));

    // Bring CS down
	GPIO_setOutputLowOnPin(SPI_CS_PORT,SPI_CS_PIN);

	ADXL362_sendSPI(0x0A);
	ADXL362_sendSPI(reg);
	ADXL362_sendSPI(TXData);

	// CS UP
	GPIO_setOutputHighOnPin(SPI_CS_PORT,SPI_CS_PIN);
}

uint8_t ADXL362_readReg(uint8_t reg){
	uint8_t aux = 0x00;

    // Bring CS down
    GPIO_setOutputLowOnPin(SPI_CS_PORT,SPI_CS_PIN);

	ADXL362_sendSPI(0x0B);
	ADXL362_sendSPI(reg);
	aux = ADXL362_rcvSPI();

    GPIO_setOutputHighOnPin(SPI_CS_PORT,SPI_CS_PIN);

    // CS UP
    return aux;  	// data
}

// b15-b14 : 00-x 01-y 10-z 11-temp
// b11-b0  : data
uint16_t ADXL362_readFIFO(){
	volatile uint16_t aux = 0;

    // Bring CS down
    GPIO_setOutputLowOnPin(SPI_CS_PORT,SPI_CS_PIN);

    ADXL362_sendSPI(0x0D);
    // each fifo sample has 16 bits, 2 bits of axis information, 2 bits redundant, 12 bits of data
	aux = ADXL362_rcvSPI();
	aux |= (ADXL362_rcvSPI() << 8);

    GPIO_setOutputHighOnPin(SPI_CS_PORT,SPI_CS_PIN);

    // CS UP
    return aux;
}

void ADXL362_beginMeasure(){
	//	1. Write 250 decimal (0xFA) to Register 0x20, and write 0 to
	//	Register 0x21: sets activity threshold to 250 mg.
	ADXL362_writeReg(THRESH_ACT_L, 0xFA);
	ADXL362_writeReg(THRESH_ACT_H, 0x00);

	//	2. Write 150 decimal (0x96) to Register 0x23, and write 0 to
	//	Register 0x24: sets inactivity threshold to 150 mg.
	ADXL362_writeReg(THRESH_INACT_L, 0x96);
	ADXL362_writeReg(THRESH_INACT_H, 0x00);
	ADXL362_writeReg(TIME_ACT, 0x02); // what to use here?

	//	3. Write 15 decimal to Register 0x25: sets inactivity
	//	timer to 15 samples or about 2 seconds.
	ADXL362_writeReg(TIME_INACT_L, 0x0F);

	//	4. Write 0x3F to Register 0x27: configures motion detection in
	//	loop mode and enables referenced activity and inactivity detection.
	ADXL362_writeReg(ACT_INACT_CTL, 0x3F);

	// config FIFO
	// set to 256 samples (watermark)
	ADXL362_writeReg(FIFO_SAMPLES, 0xFF);
	// AH set - 512 samples, oldest mode
	ADXL362_writeReg(FIFO_CTL, 0x09);

	// INT1 = Fifo wat; INT2 = AWAKE
	ADXL362_writeReg(INTMAP1, 0x84); // FIFO watermark
	ADXL362_writeReg(INTMAP2, 0xC0); // awake

	// set 2g, ODR to 50hz
	ADXL362_writeReg(FILTER_CTL, 0x11); // output data rate?

	//	6. Write 0x0E to Register 0x2D: begins the measurement in wake-up mode + autosleep disabled
	//ADXL362_writeReg(POWER_CTL, 0x0E);
	ADXL362_writeReg(POWER_CTL, 0x02);
}

void ADXL362_emptyFIFO(){
	int16_t i = 0;

    // Read whole FIFO
    GPIO_setOutputLowOnPin(SPI_CS_PORT,SPI_CS_PIN);

    ADXL362_sendSPI(0x0D);

	for (i = 0; i < 512; i++) ADXL362_rcvSPI();

    GPIO_setOutputHighOnPin(SPI_CS_PORT,SPI_CS_PIN);
}


int8_t processFIFO(){
	volatile int16_t fifo_buffer_X[170];
	volatile int16_t fifo_buffer_Y[170];
	volatile int16_t fifo_buffer_Z[170];

	uint8_t i = 0;
	int8_t axis = 0;
	int8_t stepCounter = 0;

	bool threshSide = false;
	static const int16_t THRESH = 1300;

    // Read whole FIFO into buffer
    GPIO_setOutputLowOnPin(SPI_CS_PORT,SPI_CS_PIN);
    ADXL362_sendSPI(0x0D);
	for (i = 0; i < 170; i++){
	    // each fifo sample has 16 bits, 2 bits of axis information, 2 bits redundant, 12 bits of data
		fifo_buffer_X[i] = ADXL362_rcvSPI();
		fifo_buffer_X[i] |= (ADXL362_rcvSPI() << 8);
		fifo_buffer_X[i] = (fifo_buffer_X[i] << 4);
		fifo_buffer_X[i] = (fifo_buffer_X[i] >> 4);

		fifo_buffer_Y[i] = ADXL362_rcvSPI();
		fifo_buffer_Y[i] |= (ADXL362_rcvSPI() << 8);
		fifo_buffer_Y[i] = (fifo_buffer_Y[i] << 4);
		fifo_buffer_Y[i] = (fifo_buffer_Y[i] >> 4);

		fifo_buffer_Z[i] = ADXL362_rcvSPI();
		fifo_buffer_Z[i] |= (ADXL362_rcvSPI() << 8);
		fifo_buffer_Z[i] = (fifo_buffer_Z[i] << 4);
		fifo_buffer_Z[i] = (fifo_buffer_Z[i] >> 4);
	}
    GPIO_setOutputHighOnPin(SPI_CS_PORT,SPI_CS_PIN);

   /*
    * Process FIFO
    *
	* 2 values over the THRESHOLD trigger the count analysis,
	* after that if 3 values under threshold are found, then we count a step
	*/
	for (i = 0; i < 170; i++){
		if (threshSide){
			switch (axis){
				case 1:
					if ((fifo_buffer_X[i] < THRESH) && (fifo_buffer_X[i-1] < THRESH)) {
						threshSide = false; // DOWNTHRESH met
						stepCounter++;
					}
					break;
				case 2:
					if ((fifo_buffer_Y[i] < THRESH) && (fifo_buffer_Y[i-1] < THRESH)) {
						threshSide = false; // DOWNTHRESH met
						stepCounter++;
					}
					break;
				case 3:
					if ((fifo_buffer_Z[i] < THRESH) && (fifo_buffer_Z[i-1] < THRESH)) {
						threshSide = false; // DOWNTHRESH met
						stepCounter++;
					}
					break;
			}
		} else {
			if ((fifo_buffer_X[i] > THRESH) && (fifo_buffer_X[i-1] > THRESH)) {
				threshSide = true; // UP_THRESHOLD met
				axis = 1;
			}
			else{
				if ((fifo_buffer_Y[i] > THRESH) && (fifo_buffer_Y[i-1] > THRESH)) {
					threshSide = true; // UP_THRESHOLD met
					axis = 2;
				}
				else{
					if ((fifo_buffer_Z[i] > THRESH) && (fifo_buffer_Z[i-1] > THRESH)) {
						threshSide = true; // UP_THRESHOLD met
						axis = 3;
					}
				}
			}
		}
	}

	// Additional filtering
	if (stepCounter < 3)
		stepCounter = 0;

    return stepCounter;
}

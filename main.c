
#include <driverlib.h>
#include "adxl362.h"

void main(void){
    // Stop watchdog timer
	WDT_A_hold(WDT_A_BASE);

	volatile uint16_t rd_val = 0;

	initGPIO();
	initCLKs();

	// Init SPI and IOs
	ADXL362_initSPI();
	ADXL362_initGPIO();

	// Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
	PMM_unlockLPM5();

	// Enable global interrupts for LPM 4.5
	__enable_interrupt();

	// Reset accel
	ADXL362_writeReg(0x1F, 0x52);  // Write to SOFT RESET, "R"
	__delay_cycles(500);  		   // Wait 0.5ms
	ADXL362_beginMeasure();

	// Clear interrupts
	s1ButtonPressed = 0;
	s2ButtonPressed = 0;
	fifoWatermark   = 0;
	GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
	GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
	GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN6);
	GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN7);

  while(1)
	{
		// Check if I need to service any interrupt buttons
		if(s1ButtonPressed)
		{
			// Create a 10ms debounce on the switch
			__delay_cycles(10000);

			// Force clear the flag again in case if switch debounced
			GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
			s1ButtonPressed = 0;
		}
		else if(s2ButtonPressed) // Interrupt button 2
		{
			// Show number of values on fifo
			rd_val = ADXL362_readReg(FIFO_ENTRIES_L);
			rd_val |= (ADXL362_readReg(FIFO_ENTRIES_H) << 8);
////		updateLCD(rd_val);

			// Create a 10ms debounce on the switch
			__delay_cycles(10000);

			// Force clear the flag again in case if switch debounced
			GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
			s2ButtonPressed = 0;
		}

		if (fifoWatermark){
			// Read fifo and update step counter
			aux_rdfifo = processFIFO();

			// Force clear the flag again in case if switch debounced
			GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN7);
			fifoWatermark = 0;
		}

		// Enter low power mode 4.5 and waits for fifo interrupt
		ctpl_enterLpm45(CTPL_ENABLE_RESTORE_ON_RESET);
		wakeupLPM45();

//		// Enter low power mode 3 and waits for the interrupt
//		__bis_SR_register(LPM3_bits | GIE); // CPU off, enable interrupts
//		__no_operation();				   // Remain in LPM3
	}
}

// GPIO Initialization
void initGPIO()
{
    // Set all GPIO pins to output low to prevent floating input
    // and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_PA, 0xFFFF);
    GPIO_setAsOutputPin(GPIO_PORT_PA, 0xFFFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_PB, 0xFFFF);
    GPIO_setAsOutputPin(GPIO_PORT_PB, 0xFFFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_PC, 0xFFFF);
    GPIO_setAsOutputPin(GPIO_PORT_PC, 0xFFFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_PD, 0xFFFF);
    GPIO_setAsOutputPin(GPIO_PORT_PD, 0xFFFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_PE, 0xFFFF);
    GPIO_setAsOutputPin(GPIO_PORT_PE, 0xFFFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, 0xFF);

    // Configure button S1 (P1.1) with Pull-Up Resister
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // Configure button S2 (P1.2) with Pull-Up Resister
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Configure LED 1 -- used with buttons
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);

    // Configure LED 2 -- used with accelerometer interrupts
    GPIO_setAsOutputPin(GPIO_PORT_P9,GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9,GPIO_PIN7);

    // Configure PJ.4 and PJ.5 as input pins for LFXIN and LFXOUT mode
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_PJ,
        GPIO_PIN4 + GPIO_PIN5,
        GPIO_PRIMARY_MODULE_FUNCTION
        );
}

void initCLKs(){
    CS_setExternalClockSource(32768,0); // (low freq, high freq)

    //Set DCO Frequency to 16mhz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6); // 8mhz

    //configure MCLK, SMCLK to be source by DCOCLK
    CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1); // 16mhz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1); // 16mhz
    CS_initClockSignal(CS_ACLK,  CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1); //ACLK=32768 HZ,
}

void wakeupLPM45(){
    initCLKs();
}

void wakeupShutdown(){
		initCLKs();

		ADXL362_initSPI();
		ADXL362_writeReg(0x1F, 0x52);  // Write to SOFT RESET, "R"
		__delay_cycles(500);  		   // Wait 0.5ms
		ADXL362_beginMeasure();
		__delay_cycles(500);
}

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 and S2 button press interrupts
 */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
    case P1IV_P1IFG1: // Button S1 pressed
        s1ButtonPressed = 1;
        // __bic_SR_register_on_exit(LPM3_bits);
        break;
    case P1IV_P1IFG2: // Button S2 pressed
        s2ButtonPressed = 1;
				// __bic_SR_register_on_exit(LPM3_bits);
        break;
    default: break;
    }
}

/*
 * PORT3 Interrupt Service Routine
 * Handles ADXL362 interrupts
 */
#pragma vector=PORT3_VECTOR
__interrupt void PORT3_ISR(void)
{
    switch(__even_in_range(P3IV, P3IV_P3IFG7))
    {
    case P3IV_P3IFG6: // INT 2
//    	accelAwake = 1;
    //	__bic_SR_register_on_exit(LPM3_bits);
    	break;
    case P3IV_P3IFG7: // INT 1
    	fifoWatermark = 1;
    	// __bic_SR_register_on_exit(LPM3_bits);
    	break;
    default: break;
    }
}

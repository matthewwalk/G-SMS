#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC802.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include <math.h>

//Input/Output Setup
#define MSM_Button (8)
#define LED_LOW (9)
#define LED_MED (7)
#define LED_HI (12)
#define SEN_IN (9)//GPIO17
#define SEN_POW (4)
#define DIS_POW (0)

//Constants
#define LOW_VAL (900)
#define HIGH_VAL (2000)

//Timer Setup
#define SYSTICK_FREQ (6000000)
#define WKT_FREQ (1000000)
#define WKT_RELOAD (10000000) // Reload value for the WKT down counter
#define WKT_INT_FREQ (WKT_FREQ/WKT_RELOAD) // Interrupt frequency of the WKT.

//i2c
#define LPC_I2C0BUFFERSize (9)
#define LPC_I2C0BAUDRate (100000)// 100kHz

#define display_I2CAddress (0x70)
#define HT16K33_BLINK_CMD (0x80)
#define HT16K33_BLINK_DISPLAYON (0x01)
#define HT16K33_BLINK_OFF (0)
#define HT16K33_CMD_BRIGHTNESS (0xE0)

// Define values for I2C registers that aren't in the header file.
// Table 195 of LPC802 User Manual
#define MSTCTL_CONTINUE (1UL << 0) // Bit 0 of MSTCTL set ("Main" or "Primary")
#define MSTCTL_START (1UL << 1) // Bit 1 of MSTCTL set ("Main" or "Primary")
#define MSTCTL_STOP (1UL << 2) // Bit 2 of MSTTCL set ("Main" or "Primary")
#define CTL_SLVCONTINUE (1UL << 0) // Bit 0: Secondary level (SLV) Continue
#define CTL_SLVNACK (1UL << 1) // Bit 1: Secondary Level (SLV) Acknowledge
#define PRIMARY_STATE_MASK (0x7<<1) // bits 3:1 of STAT register for Main / Primary
#define I2C_STAT_MSTST_IDLE ((0b000)<<1) // Main Idle: LPC802 user manual table 187
#define I2C_STAT_MSTST_RXRDY ((0b001)<<1) // Main Receive Ready " "
#define I2C_STAT_MSTST_TXRDY ((0b010)<<1) // Main Transmit Ready " "
#define I2C_STAT_MSTST_NACK_ADD ((0b011)<<1)// Main Ack Add " "
#define I2C_STAT_MSTST_NACK_DATA ((0b100)<<1)// Main Ack signal data ” ”

//Prototypes of Functions for i2c
void WaitI2CPrimaryState(I2C_Type * ptr_LPC_I2C, uint32_t state);
void I2C_PrimarySetBaudRate(uint32_t baudRate_Bps, uint32_t srcClock_Hz);
void display_i2c_init(void);
void display_read(float *data, uint8_t *orgdatabuffer);

//Other Prototypes
void WKT_Config();
void SysTick_Config1();
void ISR_setup();
void init_ADC(void);
void read();
void store();
void display();
void GPIO_setup();
void i2c_init();
void clear_disp();
void display_int(int val);

//sensor value
uint32_t volatile sen_val;

static const uint8_t numbertable[] = {
	0x3F, /* 0 */
	0x06, /* 1 */
	0x5B, /* 2 */
	0x4F, /* 3 */
	0x66, /* 4 */
	0x6D, /* 5 */
	0x7D, /* 6 */
	0x07, /* 7 */
	0x7F, /* 8 */
	0x6F, /* 9 */
	0x77, /* a */
	0x7C, /* b */
	0x39, /* C */
	0x5E, /* d */
	0x79, /* E */
	0x71, /* F */
};

int main(void) {
	sen_val = 0;

	__disable_irq(); // turn off globally
	SysTick_Config1();
	WKT_Config();
	GPIO_setup();
	ISR_setup();
	init_ADC();
	i2c_init();
	__enable_irq(); // turn on global

	read();

    while(1) {
        __asm("NOP");
    }
    return 0 ;
}

void read() {
	GPIO->SET[0] = (1UL<<SEN_POW); //turn on sensor
	ADC0->SEQ_CTRL[0] |= (1UL << SEN_IN); //turn on ADC channel 9.

	//Step 2. Set TRIGPOL to 1 and SEQ_ENA to	1	in	SEQA_CTRL	register
	ADC0->SEQ_CTRL[0] |= (1UL << ADC_SEQ_CTRL_TRIGPOL_SHIFT);//	trig	pol	set	to	1.
	ADC0->SEQ_CTRL[0] |= (1UL << ADC_SEQ_CTRL_SEQ_ENA_SHIFT);//	Sequence	A	turned	ON.

	//Step	6.	read	result	bits	in	DAT2	(data	for	channel	2)	for	conversion	result.
	ADC0->SEQ_CTRL[0] &= ~(1UL << ADC_SEQ_CTRL_START_SHIFT);//	start	bit	to	0
	ADC0->SEQ_CTRL[0] |= (1UL << ADC_SEQ_CTRL_START_SHIFT);	//	start	bit	to	1

	//Read	the	captured	value	on	ADC	ch	2.	Assign	it	to	a	variable.
	sen_val = ((ADC0->DAT[SEN_IN]) & (ADC_DAT_RESULT_MASK));//	isolate	bits	15:4	(data)
	sen_val = (sen_val >> ADC_DAT_RESULT_SHIFT);//	shift	right;get	true	numeric	value.

	ADC0->SEQ_CTRL[0] &= ~(1UL << SEN_IN); //turn off ADC channel 9
	GPIO->CLR[0] = (1UL<<SEN_POW); //turn off sensor
}

void store() {

}

void display() {
	//power the display
	GPIO->SET[0] = (1UL<<DIS_POW);
	for (int i = 0; i < 100; i++) asm("NOP"); //wait some time for display to recieve power

	//turn on oscillator
	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_IDLE); // Wait for the Primary's state to be idle
	I2C0->MSTDAT = (display_I2CAddress<<1) | 0; // Address with 0 for RWn bit (WRITE)
	I2C0->MSTCTL = MSTCTL_START; // Start the transaction

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK'd
	I2C0->MSTDAT = 0x21;
	I2C0->MSTCTL = MSTCTL_CONTINUE;

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
	I2C0->MSTCTL = MSTCTL_STOP;
	//-----------------------------------

	//set blinkrate
	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_IDLE); // Wait for the Primary's state to be idle
	I2C0->MSTDAT = (display_I2CAddress<<1) | 0; // Address with 0 for RWn bit (WRITE)
	I2C0->MSTCTL = MSTCTL_START; // Start the transaction

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK'd
	I2C0->MSTDAT = (HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | HT16K33_BLINK_OFF);
	I2C0->MSTCTL = MSTCTL_CONTINUE;

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
	I2C0->MSTCTL = MSTCTL_STOP;
	//-----------------------------------------

	//set brightness
	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_IDLE); // Wait for the Primary's state to be idle
	I2C0->MSTDAT = (display_I2CAddress<<1) | 0; // Address with 0 for RWn bit (WRITE)
	I2C0->MSTCTL = MSTCTL_START; // Start the transaction

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK'd
	I2C0->MSTDAT = (HT16K33_CMD_BRIGHTNESS | 5);
	I2C0->MSTCTL = MSTCTL_CONTINUE;

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
	I2C0->MSTCTL = MSTCTL_STOP;
	//----------------------------------------

	int numberArray[4];
	int n = sen_val;
	for (int c = 3; c > -1; c--) {
		numberArray[c] = n % 10;
		if (n != 0) n /= 10;
	}

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_IDLE); // Wait for the Primary's state to be idle
	I2C0->MSTDAT = (display_I2CAddress<<1) | 0; // Address with 0 for RWn bit (WRITE)
	I2C0->MSTCTL = MSTCTL_START; // Start the transaction

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK'd
	I2C0->MSTDAT = 0x00;
	I2C0->MSTCTL = MSTCTL_CONTINUE;

	int pos = 0;
	for (uint8_t i = 0; i < 5; i++) {
		if (i == 2) {
			WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
			I2C0->MSTDAT = 0x00 & 0xFF;  //<--- WHAT TO SEND
			I2C0->MSTCTL = MSTCTL_CONTINUE;

			WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
			I2C0->MSTDAT = 0x00 >> 8;  //<--- WHAT TO SEND
			I2C0->MSTCTL = MSTCTL_CONTINUE;
		}
		else {
			WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
			I2C0->MSTDAT = numbertable[numberArray[pos]] & 0xFF;  //<--- WHAT TO SEND
			I2C0->MSTCTL = MSTCTL_CONTINUE;

			WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY);
			I2C0->MSTDAT = numbertable[numberArray[pos]] >> 8;  //<--- WHAT TO SEND
			I2C0->MSTCTL = MSTCTL_CONTINUE;
			pos++;
		}
	}

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK’d
	I2C0->MSTCTL = MSTCTL_STOP;
	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_IDLE);

	for (int i = 0; i < 1000000; i++) asm("NOP"); //display for a second
	GPIO->CLR[0] = (1UL<<DIS_POW);
}

void i2c_init() {
    // enable switch matrix.
    SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
    // Set switch matrix
    SWM0->PINASSIGN.PINASSIGN5 &= ~(SWM_PINASSIGN5_I2C0_SCL_IO_MASK |
    SWM_PINASSIGN5_I2C0_SDA_IO_MASK); // clear 15:0
    SWM0->PINASSIGN.PINASSIGN5 |= ((16<<SWM_PINASSIGN5_I2C0_SCL_IO_SHIFT)| // Put 16 in bits 15:8
    (10<<SWM_PINASSIGN5_I2C0_SDA_IO_SHIFT)); // put 10 in bits 7:0
    // disable the switch matrix
    SYSCON->SYSAHBCLKCTRL0 &= ~(SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
    // ------------------------------------------------------------
    // Step 2: Turn on the I2C module via the SYSCON clock enable pin
    // ------------------------------------------------------------
    SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_I2C0_MASK );// I2C is on
    // Put 0 in the GPIO, GPIO Interrupt and I2C reset bit to reset it.
    // Then put a 1 in the GPIO, GPIO Interrupt and I2C reset bit to allow them to operate.
    SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_I2C0_RST_N_MASK );// reset I2C(bit = 0)
    SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_I2C0_RST_N_MASK);// remove i2C reset (bit = 1)
    // Provide FRO as function clock to I2C0
    SYSCON->I2C0CLKSEL = 0b000;
    // ------------------------------------------------------------
    // Step 4: enable primary (but not slave) functionality in the i2c module.
    // ------------------------------------------------------------
    // Only config I2C0 as a primary
    I2C0->CFG = (I2C_CFG_MSTEN_MASK); // only as primary

    // Set i2C bps to 100kHz assuming an input clock of 12MHz
    I2C_PrimarySetBaudRate(100000, 12000000);
}

void WaitI2CPrimaryState(I2C_Type * ptr_LPC_I2C, uint32_t state) {
	while(!(ptr_LPC_I2C->STAT & I2C_STAT_MSTPENDING_MASK)); // Wait
	// Check to see that the state is correct.
	// if it is not, then turn on PIO0_9 to indicate a problem
	// Primary's state is in bits 3:1. PRIMARY_STATE_MASK is (0x7<<1)
	if((ptr_LPC_I2C->STAT & PRIMARY_STATE_MASK) != state) // If primary state mismatch
	{
	while(1); // die here and debug the problem
	}
	return; // If no mismatch, return
}

void I2C_PrimarySetBaudRate(uint32_t baudRate_Bps, uint32_t srcClock_Hz) {
	uint32_t scl, divider;
	uint32_t best_scl, best_div;
	uint32_t err, best_err;
	best_err = 0;
	for (scl = 9; scl >= 2; scl--) {
		/* calculated ideal divider value for given scl */
		divider = srcClock_Hz / (baudRate_Bps * scl * 2u);
		/* adjust it if it is out of range */
		divider = (divider > 0x10000u) ? 0x10000 : divider;
		/* calculate error */
		err = srcClock_Hz - (baudRate_Bps * scl * 2u * divider);
		if ((err < best_err) || (best_err == 0)) {
			best_div = divider;
			best_scl = scl;
			best_err = err;
		}
		if ((err == 0) || (divider >= 0x10000u)) {
			/* either exact value was found
			 or divider is at its max (it would even greater in the next iteration for sure) */
			break;
		}
	}
	// Assign Clock Divider value, using included in LPC802.h
	I2C0->CLKDIV = I2C_CLKDIV_DIVVAL(best_div - 1);
	// Assign Primary timing configuration, using two macros include in LPC802.h
	I2C0->MSTTIME =
	I2C_MSTTIME_MSTSCLLOW(best_scl - 2u) | I2C_MSTTIME_MSTSCLHIGH(best_scl - 2u);
}

void init_ADC() {
//	---------------------------------------------------
//	Step	1.	Power	the	ADC	with	PDRUNCFG
	SYSCON->PDRUNCFG &= ~(SYSCON_PDRUNCFG_ADC_PD_MASK);
//	---------------------------------------------------
//	Step	2.	SYSAHBCLKCTRL	to	enable	clock	register	interfaces:
	SYSCON->SYSAHBCLKCTRL0 |= ( SYSCON_SYSAHBCLKCTRL0_ADC_MASK |
	SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
//	Reset	the	ADC	module.
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_ADC_RST_N_MASK); //	Assert reset	(0)
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_ADC_RST_N_MASK); //	Remove	reset(1)
//	---------------------------------------------------
//	Step	3.	Use	ADCASYNCCLKSEL	and	ADCASYNCDIV	to	control	ADC	clock
	SYSCON->ADCCLKSEL &= ~(SYSCON_ADCCLKSEL_SEL_MASK);//	Use	fro_clk	as	source	for	ADC	async clock
//	Divide	the	FRO	clock	into	the	ADC.		If	0	it	shuts	down	the	ADC	clock?
	SYSCON->ADCCLKDIV = 1;	//	divide	by	1	(values:	0	to	255)
//	---------------------------------------------------
//	Step	5.	Enable	a	particular	ADC	channel
	SWM0->PINENABLE0 &= ~(SWM_PINENABLE0_ADC_9_MASK);
}

void ISR_setup() {
	 // disable interrupts
	 NVIC_DisableIRQ(PIN_INT0_IRQn); // turn off the PIN INT0 interrupt.
	 // Set up GPIO IRQ: interrupt channel 0 (PINTSEL0) to GPIO 8
	 SYSCON->PINTSEL[0] = MSM_Button; // PINTSEL0 is P0_8
	 // Configure the Pin interrupt mode register (a.k.a ISEL) for edge-sensitive
	 // on PINTSEL0. 0 is edge sensitive. 1 is level sensitive.
	 PINT->ISEL = 0x00; // channel 0 bit is 0: is edge sensitive (so are the other channels)
	 // Use IENR or IENF (or S/CIENF or S/CIENR) to set edge type
	 // Configure Chan 0 for only falling edge detection (no rising edge detection)
	 PINT->CIENR = 0b00000001; // bit 0 is 1: disable channel 0 IRQ for rising edge
	 PINT->SIENF = 0b00000001; // bit 0 is 1: enable channel 0 IRQ for falling edge
	 // Remove any pending or left-over interrupt flags
	 PINT->IST = 0xFF; // each bit set to 1 removes any pending flag.
	 // enable global interrupts & GPIO INT channel 0
	 NVIC_EnableIRQ(PIN_INT0_IRQn); // GPIO interrupt
}

void GPIO_setup() {
	 // ----------------------- Begin GPIO setup ------------------------------------
	 SYSCON->SYSAHBCLKCTRL0 |= ( SYSCON_SYSAHBCLKCTRL0_GPIO0_MASK | // GPIO is on
	 SYSCON_SYSAHBCLKCTRL0_GPIO_INT_MASK); // GPIO Interrupt is on
	 SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_GPIO0_RST_N_MASK |
	 SYSCON_PRESETCTRL0_GPIOINT_RST_N_MASK);// reset GPIO and GPIO Interrupt (bit=0)
	 SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_GPIO0_RST_N_MASK |
	 SYSCON_PRESETCTRL0_GPIOINT_RST_N_MASK);// clear reset (bit=1)
	 GPIO->CLR[0] = (1UL<<LED_LOW);
	 GPIO->CLR[0] = (1UL<<LED_MED);
	 GPIO->CLR[0] = (1UL<<LED_HI);
	 GPIO->CLR[0] = (1UL<<SEN_POW);
	 GPIO->CLR[0] = (1UL<<DIS_POW);
	 GPIO->DIRCLR[0] = (1UL<<MSM_Button); // input on PB8 (BUTTON_USER1)
	 GPIO->DIRSET[0] = (1UL<<LED_LOW);
	 GPIO->DIRSET[0] = (1UL<<LED_MED);
	 GPIO->DIRSET[0] = (1UL<<LED_HI);
	 GPIO->DIRSET[0] = (1UL<<SEN_POW);
	 GPIO->DIRSET[0] = (1UL<<DIS_POW);
	 // ----------------------- end of GPIO setup -----------------------------------
}

void WKT_Config() {
	//turn on wkt
	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_WKT_MASK);
	//disable interrupts
	NVIC_DisableIRQ(WKT_IRQn);
	//turn on the lpo power supply
	SYSCON->PDRUNCFG &= ~(SYSCON_PDRUNCFG_LPOSC_PD_MASK);
	//enable the clock to the wkt
	SYSCON->LPOSCCLKEN |= (SYSCON_LPOSCCLKEN_WKT_MASK);
	//reset and clear reset of wkt
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_WKT_RST_N_MASK);
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_WKT_RST_N_MASK);
	WKT->CTRL = (WKT_CTRL_CLKSEL_MASK); // (Choose Low Power Clock using Bit 0)
	WKT->COUNT = (WKT_RELOAD);
	NVIC_EnableIRQ(WKT_IRQn);
}

void SysTick_Config1() {
	NVIC_DisableIRQ(SysTick_IRQn); //systick specific
	// Specify that we will use the Free-Running Oscillator
	SYSCON->MAINCLKSEL = (0x0 << SYSCON_MAINCLKSEL_SEL_SHIFT);
	// Update the Main Clock
	SYSCON->MAINCLKUEN &= ~(0x1);
	SYSCON->MAINCLKUEN |= (0x1);
	SysTick_Config(SYSTICK_FREQ); //1sec
	NVIC_EnableIRQ(SysTick_IRQn); // SysTick IRQs are on.
}

void SysTick_Handler(void) {
	if (sen_val <= LOW_VAL) {
		GPIO->SET[0] = (1UL<<LED_LOW);
		GPIO->CLR[0] = (1UL<<LED_MED);
		GPIO->CLR[0] = (1UL<<LED_HI);
	} else if  (sen_val >= HIGH_VAL) {
		GPIO->CLR[0] = (1UL<<LED_LOW);
		GPIO->CLR[0] = (1UL<<LED_MED);
		GPIO->SET[0] = (1UL<<LED_HI);
	} else {
		GPIO->CLR[0] = (1UL<<LED_LOW);
		GPIO->SET[0] = (1UL<<LED_MED);
		GPIO->CLR[0] = (1UL<<LED_HI);
	}
}

void WKT_IRQHandler(void) {
	WKT->CTRL |= (1<<WKT_CTRL_ALARMFLAG_SHIFT); // Clear the interrupt flag
	WKT->COUNT = WKT_RELOAD; // clear
	//request reading
	read();
	//store
	store();
}

void PIN_INT0_IRQHandler(void) {
	// was an IRQ requested for Channel 0 of GPIO INT?
	if (PINT->IST & (1 << 0)) {
		// remove the any IRQ flag for Channel 0 of GPIO INT
		PINT->IST = (1 << 0);
		//Request Reading
		read();
		//Display Value
		display();
	} else {
		__asm("NOP");
	}
	return;
}

/**
		File name: main.c
		Theme : Home Automation
		Functions: 
			setup()
			setupUART()
			switchPinConfig()
			ledPinConfig()
			enableTimerInterrupt()
			configureADCSequencer()
			UA(RTputCharacters()
			configurePWM()
			adjustFanByTemperature()
			takeAction()
			getTemperature()
			detectMotion()
			detectSecurity()
			Timer0IntHandler()

		Global Variables: 
			PWM_FREQUENCY
			ui32Adjust
			ui32Load
			FAN_INCREMENT
			FAN_LOW
			FAN_HIGH
			ui32ADC0Value
			ui32TempAvg
			ui32TempValueC
			ui32TempValueF
			ui32NormalTemp
			bluetoothInput
			motionState
			securityState
			iter
 * */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#define TARGET_IS_BLIZZARD_RB1

#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"

// LOCK_F and CR_F - used for unlocking PORTF pin 0
#define LOCK_F (*((volatile unsigned long *)0x40025520))
#define CR_F   (*((volatile unsigned long *)0x40025524))

#define PWM_FREQUENCY 55

/*
	Function Name: setup
	Input: 
	Output: 
	Logic: 
		Enables critical peripherals to use later like
			PORT D, E, F, B
			ADC
			UART
		enables ADC sequences
	Example Call: setup()	
*/
void setup(void)
{
	// Setup the frequency of Microcontroller to be 40MHz
//	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
//	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	// Enable Perpheral GPIOD for inputs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	// Enable perepheral GPIOF for LEDs and Switches
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// EnableADC Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	// Enable peripheral for UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH5|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE,2,0,ADC_CTL_CH4|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 2);
}

/*
	Function Name: setupUART()
	Input:
	Output:
	Logic: 
		Set the clocking to directly run from the crystal at 8MHz, 
		sets up UART 1 to configure Port B0 as RX and Port B1 as TX
	example: setupUART()			
*/
void setupUART(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);

	/* Make the UART pins be peripheral controlled. */
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART1_BASE,16000000, 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

/*
	Function Name: switchPinConfig
	Input:
	Output:
	Logic: 
		sets Port D Pin 2 and 3 as ADC input for temperature control
		removes lock from SW2
		sets PORTF Pin 0 and Pin4 as input		
*/
void switchPinConfig(void)
{
	// GPIO PORTD Pins to be set as ADC input
	GPIODirModeSet(GPIO_PORTD_BASE,GPIO_PIN_2|GPIO_PIN_3,GPIO_DIR_MODE_IN); // Set Pin-4 of PORT F as Input. Modifiy this to use another switch
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_2|GPIO_PIN_3,GPIO_STRENGTH_12MA,GPIO_PIN_TYPE_STD_WPU);

	// Following two line removes the lock from SW2 interfaced on PORTF Pin0 -- leave this unchanged
	LOCK_F=0x4C4F434BU;
	CR_F=GPIO_PIN_0|GPIO_PIN_4;

	// GPIO PORTF Pin 0 and Pin4
	GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_DIR_MODE_IN); // Set Pin-4 of PORT F as Input. Modifiy this to use another switch
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_STRENGTH_12MA,GPIO_PIN_TYPE_STD_WPU);

	GPIODirModeSet(GPIO_PORTE_BASE,GPIO_PIN_5|GPIO_PIN_3,GPIO_DIR_MODE_IN);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5|GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_5|GPIO_PIN_3,GPIO_STRENGTH_12MA,GPIO_PIN_TYPE_STD_WPU);
}

/*
	Function Name: ledPinConfig
	Logic: Sets Pin for LEDs i.e. Port F pin 1,2,3 as Output Pins
*/
void ledPinConfig(void)
{
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

/*
	Function Name: enableTimerInterrupt
	Logic: Enables Timer A Interrupt, sets the interrupt period
*/
void enableTimerInterrupt(void)
{
	uint32_t ui32Period;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	ui32Period = (SysCtlClockGet() * 4) / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);
}

/*
	Function Name: configureADCSequencer
	Logic: 
		Configures ADC sequence Steps to sample Temperature Sensors
*/
void configureADCSequencer(void)
{
//	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64);
	ROM_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
	ROM_ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
	ROM_ADCSequenceEnable(ADC0_BASE, 1);
}

/*
	Function Name: UARTputCharacters
	Input : output char*(string to be printed)
	Logic: sends the string to TX pin of UART one character at a time
	Example Call: UARTputCharacters("Hello World")
*/
void UARTputCharacters(char* output)
{
	int i;
	for (i = 0; i < strlen(output); i++)
	{
		UARTCharPut(UART1_BASE, output[i]);
	}
	UARTCharPut(UART1_BASE,'\r');
	UARTCharPut(UART1_BASE,'\n');
}

#define FAN_INCREMENT 50
#define FAN_LOW 200
#define FAN_HIGH 1000


volatile uint32_t ui32Adjust;
volatile uint32_t ui32Load;
/*
	Function name: configurePWM
	Logic: Configures PWM on Port F pin 1,2,3 and Port E pin 4 for LED and fan control
*/
void configurePWM(void)
{
	volatile uint32_t ui32PWMClock;

	ui32Adjust = FAN_LOW;

//	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);

	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PE4_M1PWM2);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui32Adjust * ui32Load / 1000);
//	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32Adjust * ui32Load / 1000);
//	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32Adjust * ui32Load / 1000);

	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT|PWM_OUT_5_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}

/*
	Variable Name: ui32ADC0Value
	Contains the 4 sampled Value for ADC input
*/
uint32_t ui32ADC0Value[4];
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;
/*
	Variable Name: ui32NormalTemp
	Stores the average room temperature. In the auto mode, this is used to calibrate fan speed
*/
volatile uint32_t ui32NormalTemp = 15;

/*
	Function Name: adjustFanByTemperature
	Adjusts Fan PWM output according to the temperature sensed   
*/
void adjustFanByTemperature(void)
{
	ui32Adjust = FAN_LOW + (FAN_HIGH - FAN_LOW)/(1 + exp(-(double)(ui32TempValueC - ui32NormalTemp)));
}

/*
* Function Name: takeAction
* Input: Input character received through bluetooth
* Output: None
* Logic: 
	Based on the input received from bluetooth controller take the action
* Example Call:
	takeAction('0') switches off LED
*/
void takeAction(unsigned char input)
{
	switch(input)
	{
		case '0':					// LED OFF
		{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);
			break;
		}
		case '1':					// RED
		{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
			break;
		}
		case '2':					// GREEN
		{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
			break;
		}
		case '3':					// BLUE
		{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
			break;
		}
		case '5':					// FAN ON
		{
			ui32Adjust = (FAN_HIGH + FAN_LOW)/2;
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
			break;
		}
		case '6':					// FAN OFF
		{
			ui32Adjust = FAN_LOW;
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
			break;
		}
		case '7':					// FAN UP
		{
			ui32Adjust += FAN_INCREMENT;
			if (ui32Adjust > FAN_HIGH)
			{
				ui32Adjust = FAN_HIGH;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
			break;
		}
		case '8':					// FAN DOWN
		{
			ui32Adjust -= FAN_INCREMENT;
			if (ui32Adjust < FAN_LOW)
			{
				ui32Adjust = FAN_LOW;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
			break;
		}
		case '9':					// FAN AUTO
		{
			adjustFanByTemperature();
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
			break;
		}
		default:
		{

		}
	}
}

// input : stores the input received from bluetooth controller
volatile unsigned char bluetoothInput;
int main(void)
{
	setup();
	setupUART();
	switchPinConfig();
	configurePWM();
	ledPinConfig();
	configureADCSequencer();
	enableTimerInterrupt();

	UARTputCharacters("HELLO WORLD");

	while(1)
	{
		while(!UARTCharsAvail(UART1_BASE));
		bluetoothInput = UARTCharGet(UART1_BASE);
		takeAction(bluetoothInput);
//		UARTCharPut(UART1_BASE,bluetoothInput);
	}
}


/*
* Function Name: getTemperature
* Input: None
* Output: None
* Logic: 
	takes input from ADC into ui32ADC0Value
	and takes average of the temperature value
	and converts the values into Celsius and Fahrenheit
* Example Call:
	getTemperature()	
*/

void getTemperature()
{
	ROM_ADCIntClear(ADC0_BASE, 1);
	ROM_ADCProcessorTrigger(ADC0_BASE, 1);
	while(!ROM_ADCIntStatus(ADC0_BASE, 1, false))
	{

	}
	ROM_ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
	ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
	ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
	ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
}

// motionState : stores the state of motion read by motion sensor
volatile uint32_t motionState = 0;

/*
* Function Name: detectMotion
* Input:
* Output: 
* Logic: 
	Read Port E pin 5 for motion sensor output
* Example Call:
	detectMotion()
*/
void detectMotion(void)
{
	if(GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_5)==0x00)
	{
		motionState = 1;
	}
	else
	{
		if (motionState == 1)
		{
			UARTputCharacters("MOTION DETECTED!");
		}
		motionState = 0;
	}
}

// securitystate : stores the state of security read by motion sensor
volatile uint32_t securityState = 0;
/*
* Function Name: detectSecurity
* Input:
* Output: 
* Logic: 
	Read Port E pin 3 for proximity sensor output
* Example Call:
	detectSecurity()
*/
void detectSecurity(void)
{
	if(GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_3)==0x00)
	{
		securityState = 1;
	}
	else
	{
		if (securityState == 1)
		{
			UARTputCharacters("THIEF ALERT!");
		}
		securityState = 0;
	}
}

// iter : keep track of the progress of the program 
volatile uint32_t iter = 0;

/*
* Function Name: Timer0IntHandler
* Input:
* Output: 
* Logic: 
	Timer 0 Interrupt Handler
	In each timer interrupt 
		temperature, motion and proximity sensors are sensed
* Example Call:
	Timer0IntHandler()

*/
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	iter++;
	getTemperature();
	detectMotion();
	detectSecurity();
}

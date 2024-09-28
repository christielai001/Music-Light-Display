#include <stdint.h>
#include <stdbool.h>
#define ARM_MATH_CM4 1 
#include "arm_math.h" 
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"


#include "inc/tm4c123gh6pm.h"

#define RED_MASK 0x02
#define BLUE_MASK 0x04
#define GREEN_MASK 0x08

// The standard udma control table
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif



// a and b buffers
#define ADC_BUF_SIZE	512
static uint32_t ui32BufA[ADC_BUF_SIZE];
static uint32_t ui32BufB[ADC_BUF_SIZE];


static uint8_t ui8Flags = 0; 
static float32_t maxTest = 0;
static float32_t freqValue;

void ADC0_Init(void)
{
	
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.
		ADCSequenceDisable(ADC0_BASE, 1); //disable ADC0 before the configuration is complete
		ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 1); //Use with TimerControlTrigger(uint32_t ui32Base,uint32_t ui32Timer,bool bEnable)
  	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END); //ADC0 SS1 Step 0, sample from channel 0
    IntPrioritySet(INT_ADC0SS1, 0x00);  	 // configure ADC0 SS1 interrupt priority as 0
		IntEnable(INT_ADC0SS1);    				// enable interrupt 31 in NVIC (ADC0 SS1)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS1);      // arm interrupt of ADC0 SS1	
		ADCSequenceEnable(ADC0_BASE, 1); //enable ADC0
}


void Timer0A_Init(void) //ADC 10KHz trigger
	
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE,TIMER_A,999);//40,000,000/40,000 - 1
	TimerControlTrigger(TIMER0_BASE, TIMER_A,true);
	IntPrioritySet(INT_TIMER0A,0x00);
	IntEnable(INT_TIMER0A);	
	TimerEnable(TIMER0_BASE,TIMER_A);
}

void DMA_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);// Enable the clock
    uDMAEnable(); // Enable the uDMA 
    uDMAControlBaseSet(ui8ControlTable); //use the ui8ControlTable

	//disable attributes of udma channel
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC1,
                                UDMA_ATTR_ALTSELECT |
                                UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);		
		
//set channel paramters of primary data structure	
    uDMAChannelControlSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                          UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                          UDMA_ARB_1);
	
//set channel paramters of alternate data structure	
    uDMAChannelControlSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
                          UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                          UDMA_ARB_1);
	
//set transfer paramters of primary data structure. uses buffer A
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO1),
                               ui32BufA, ADC_BUF_SIZE);
															 
//set transfer paramters of alternate data structure. uses buffer B
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO1),
                               ui32BufB, ADC_BUF_SIZE);
		//enables the channel
    uDMAChannelEnable(UDMA_CHANNEL_ADC1);											 
}



void ADC0_Handler(void)
{
	//ADC ISR
		
	//clear SS1
		ADCIntClear(ADC0_BASE, 1);
	//create int variable to store current DMA mode
		uint32_t ui32Mode;
	
	//get current mode (ping pong mode) of primary data structure
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT);
	
	//when transfer is complete, mode is STOP
    if(ui32Mode == UDMA_MODE_STOP)
    {
				ui8Flags |= 0x01; 
			
				//reload DMA and set mode to pingpong
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
						UDMA_MODE_PINGPONG,
						(void *)(ADC0_BASE + ADC_O_SSFIFO1),
						ui32BufA, ADC_BUF_SIZE);
						
				//enable the DMA channel											 
				uDMAChannelEnable(UDMA_CHANNEL_ADC1);
    }
		
		
		//get current mode (ping pong mode) of alternate data structure
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT);
		
		
		//when transfer is complete, mode is STOP
    if(ui32Mode == UDMA_MODE_STOP)
    {
				ui8Flags |= 0x02; 
			
				//reload DMA and set mode to pingpong
				uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
						UDMA_MODE_PINGPONG,
						(void *)(ADC0_BASE + ADC_O_SSFIFO1),
						ui32BufB, ADC_BUF_SIZE);
						
				//enable the uDMA channel											 
				uDMAChannelEnable(UDMA_CHANNEL_ADC1);
    }
		
		
}


void dataProcess(uint32_t *data){
	//create array of floats because ARM functions only accept float values
	static float32_t testInput[512];
	
	float32_t ave = 0;
		for (uint16_t i = 0; i<512; i++){ 
			ave+=data[i];
			//multiply by hanning window 
			//https://www.mathworks.com/help/signal/ref/hann.html
			//data[i] = (0.5*(1- cos ( 2.0 * PI * i/511)))*data[i];
			
			
			//fill float array with int data
			testInput[i] = data[i];
			
			
			
		}
		
	//calculate average in case its needed
	ave = (ave)/512;	

	//create output array data for values in frequency domain
	static float32_t testOutput[512];
	uint32_t ifftFlag = 0;
	uint32_t doBitReverse = 1;
	arm_cfft_instance_f32 varInstCfftF32;
	uint32_t testIndex = 0;

	
	//arm_status status;
  float32_t maxValue;
	
	//find fft of values to change to frequency domain
  arm_cfft_f32(&varInstCfftF32, testInput, ifftFlag, doBitReverse);
  
	//find magnitude of the complex values
  arm_cmplx_mag_f32(testInput, testOutput, 256);
	
	//find the peak value and the index where the peak occurs
  arm_max_f32(testOutput, 256, &maxValue, &testIndex);
 
	//store peak value
	//set global value so we can check it in the main function
	maxTest = maxValue;
	
	//frequency value should be determined by index of peak, multiplied by bin length but it doesn't seem to be working?
	freqValue = testIndex*256;


}

void PortFunctionInit(void) //GPIOF and PWMs
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
   
		GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3);
	
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable pin PF1 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Enable pin PF3 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

	
		
}



int main(void)
{
		//400*100 =40khz
	  ADC0_Init();
	  DMA_Init();
		Timer0A_Init();	
	  PortFunctionInit(); 
	
		while(1)
		{
			 if ((ui8Flags&0x01) != 0) {dataProcess(ui32BufA);ui8Flags&=~0x01;}
			 if ((ui8Flags&0x02) != 0) {dataProcess(ui32BufB);ui8Flags&=~0x02;}		
			 
						if (maxTest<3000) // LEDs off
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
            }
            else if (maxTest<3050) // Red
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
            }
            else if (maxTest<3100) // Blue
            {
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
					   
            }
            else if (maxTest<3150) // Purple
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
            }
						else if (maxTest<3200) // Green
            {
								GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
							
            }
            else if (maxTest<3250) // Yellow (Red and Green)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
							
            }
            else if (maxTest<3300) // Cyan (Blue and Green)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
							
            }
            else if (maxTest<3350) // White (all LED on)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
            }			 
			 
		}
		
}



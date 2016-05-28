/***********************************************************************************************************************
 * Description:
 *      In this example MCP9700 (Analog Temperature Sensor) is connected to PIC microcontroller
 *      Any AN (Analog) pin can be used
 * Ankit
 **********************************************************************************************************************/

// include header files
#include <plib.h>

// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 80 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC1/EMUD1 pins shared with PGC1/PGD1)
#define SYS_FREQ (80000000L)


/******************************************Define setup parameters for OpenADC10 function******************************/

// Turn module on | Ouput in integer format | Trigger mode auto | Enable autosample
#define config1     ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
// ADC ref external | Disable offset test | Disable scan mode | Perform 2 samples | Use dual buffers | Use alternate mode
#define config2     ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_16 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF
// Use ADC internal clock | Set sample time
#define config3     ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
// Do not assign channels to scan
#define configscan  SKIP_SCAN_ALL
// AN3 = Temperature Sensor
#define configport  ENABLE_AN3_ANA


#define	GetPeripheralClock()                (SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define NO_OF_SAMPLES                       16

/**************************************************Global Variables****************************************************/

unsigned short tempSensor[16] ;       
unsigned int offset,i,average,actualvalue;                
long count,total;
float temperature;

//********************************************************************************************************************//
//  main    Main entry of the code
//********************************************************************************************************************//

int main(void)
{
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states and
    // enable prefetch cache but will not change the PBDIV. The PBDIV value
    // is already set via the pragma FPBDIV option above..

    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    CloseADC10();   // Ensure the ADC is off before setting the configuration

    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN3);

    // Configure ADC using the parameters defined above
    OpenADC10( config1, config2, config3, configport, configscan );
    
    EnableADC10();                                                                  // Enable the ADC

    while (1)
    {
        while ( !mAD1GetIntFlag() )
        {
            // Wait for the first conversion to complete so there will be vaild data in ADC result registers
        }

        for(i=0 ; i < NO_OF_SAMPLES ; i++)
        {
            // Read the result of Temperature sensor conversion from the buffer
            tempSensor[i] = ReadADC10(i);
            total += tempSensor[i];
        }

        average = (total/NO_OF_SAMPLES);

        // We have used 3.3v as reference for this example
        actualvalue = ((average*3.3*1000)/1023);         // we get value mv
        
        //temp = (Vout- Vzerodegree)/coefficient) 
        //output voltage at 0°C is scaled to 500 mV
        //perature coefficient of 10.0 mV/°C
        // refer MCP9700 datasheet
        temperature = (actualvalue-500)/10;
        
        total = 0;
        average = 0;
    }

    while(1);
}



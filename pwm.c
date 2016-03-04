/***********************************************************************************************************************
*   Description:
*       In this example Piezo BUzzer is connected to PIC microcontroller on OC1 module
*       
***********************************************************************************************************************/

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

// Macros
#define SYS_FREQ            (80000000L)             // (= 80 MHz)
#define TIMER_FREQUENCY     (SYS_FREQ/8)            // 8 is the prescalar for timer( = 10 MHz)
#define PWM_FREQUENCY       10000                   // PWM frequency in Hertz( we will drive pwm at 10KHz frequency)
#define PERIOD              (TIMER_FREQUENCY/PWM_FREQUENCY)

/***********************************************************************************************************************
*   main
*       Main entry routine
***********************************************************************************************************************/
int main(void)
{
    unsigned int i;
    
    /* Open Timer2 with Period register value */
    OpenTimer2(T2_ON | T2_PS_1_8 , PERIOD);

    
    /* Enable OC | Timer2 is selected | Continuous O/P | OC Pin High | Secondary compare value | Primary compare value*/
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0 ,0 );

    for( i = 0 ; i < PERIOD ; i++  )
    {
        SetDCOC1PWM(i);                                         // Writing duty cycle register
    }
    
    while(1);
}

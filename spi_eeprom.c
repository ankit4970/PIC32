/***********************************************************************************************************************
*   Description:
*       In this example 25AA010 (Serial EEPROM-SPI) is connected to PIC microcontroller on SPI1
***********************************************************************************************************************/

// include header files
#include <plib.h>
#include <p32xxxx.h>

// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 80 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
                                    // see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config ICESEL = ICS_PGx2    // ICE/ICD Comm Channel Select (pins 4,5)
#pragma config FSOSCEN = OFF        // Disable Secondary Oscillator


// Macros
#define SS              LATAbits.LATA6      // slave select pin
#define SYSCLK          40000000

// 25AA010 EEPROM instructions
#define WREN    0x06    // write enable
#define WRDI    0x04    // write disable
#define WRITE   0x02    // initialize start of write sequence
#define READ    0x03    // initialize start of read sequence
#define CE      0xc7    // erase all sectors of memory
#define RDSR    0x05    // read STATUS register

//*************************************** Function Prototypes***********************************************************
char SPI1_transfer( char b);
int waitBusy();
int eraseEEPROM();
int readEEPROM( unsigned short address, char* loadArray, unsigned short loadArray_size);
int writeEEPROM( unsigned short address, char* storeArray, unsigned short storeArray_size);


/***********************************************************************************************************************
*   SPI1_transfer
*       write to and read from SPI1 buffer
***********************************************************************************************************************/
char SPI1_transfer( char byte)
{
    SPI1BUF = byte;                        // write to buffer for TX
    while( !SPI1STATbits.SPIRBF);       // wait transfer complete
    return SPI1BUF;                     // read the received value
} 


/***********************************************************************************************************************
*   waitBusy
*       checks if EEPROM is ready to be modified and waits if not ready
***********************************************************************************************************************/
int waitBusy()
{
    char status = 0;

    do{
        SS = 0;                         // Select EEPROM
        SPI1_transfer( RDSR);           // Read EEPROM status register
        status = SPI1_transfer( 0);     // send dummy byte to receive incoming data
        SS = 1;                         // Release EEPROM
    }while( status & 0x01);             // write-in-progress while status<0> set to '1'

    return 0;
}                           



/***********************************************************************************************************************
*   main
*       Main entry routine
***********************************************************************************************************************/
int main()
{
    // optimize PIC32 performance and guarantee correct system clock frequency
    SYSTEMConfigPerformance(SYSCLK);

    unsigned char readData[10]= {};             // Array to read data bytes

    TRISAbits.TRISA6 = 0;                       // RA6 (Slave Select 1)

    // SPI config
    // CKP (clock polarity control) = 0
    // CKE (clock edge control) = 1
    // 8-bit, Master Mode
    // Baud = 4MHz = Fpb/( 2*( SPI1BRG+1)) = 40MHz/( 2*( 4+1))
    SPI1CON = 0x8120;
    SPI1BRG = 0x0004;

    //*********************write operation************************//
    
    SS = 0;                                     // Select EEPROM
    SPI1_transfer( WREN);                       // Send WRITE_ENABLE command
    SS = 1;                                     // Release EEPROM
    
    SS = 0;                                     // Select EEPROM again after WREN cmd
    
    // Writing 10 bytes
    SPI1_transfer( WRITE);                      // Initiate write
    SPI1_transfer( 0x02 );                      // Address
    SPI1_transfer( 0x11);                       // Data Byte 1
    SPI1_transfer( 0x22);
    SPI1_transfer( 0x33);
    SPI1_transfer( 0x44);
    SPI1_transfer( 0x55);
    SPI1_transfer( 0x66);
    SPI1_transfer( 0x77);
    SPI1_transfer( 0x88);
    SPI1_transfer( 0x99);
    SPI1_transfer( 0x00);                       // Data Byte 10

    SS = 1;                                     // Release EEPROM

    waitBusy();                                 // Wait until EEPROM is not busy
    
    //*************************************end of write operation**************************************//


    
    //*************************************read operation**********************************************//

    
    waitBusy();                                 // Wait until EEPROM is not busy

    SS = 0;                                     // Select EEPROM
    
    SPI1_transfer( READ);                       // initiate read
    SPI1_transfer( 0x02);
    readData[0] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[1] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[2] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[3] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[4] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[5] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[6] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[7] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[8] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    readData[9] = SPI1_transfer( 0);            // send dummy byte to read 1 byte
    
    SS = 1;                                     // Release EEPROM
    //****************************************end of read***********************************************//

}

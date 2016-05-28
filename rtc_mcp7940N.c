/***********************************************************************************************************************
 * Description:
 *      In this example MCP7940N (I2C Real Time Clock) is connected to PIC microcontroller
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
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_8
#define SYS_FREQ (80000000L)

/*****************************************************Macros***********************************************************/

#define GetSystemClock()           (SYS_FREQ)
#define GetPeripheralClock()       (SYS_FREQ/8)
#define GetInstructionClock()      (SYS_FREQ)
#define I2C_CLOCK_FREQ             100000

/***********************************************RTC Constants*******************************************************/

#define RTC_BUS                 I2C2
#define RTC_ADDRESS             0x6F        // 0b01101111

#define  ADDR_SEC          0x00       //  address of SECONDS      register
#define  ADDR_MIN          0x01       //  address of MINUTES      register
#define  ADDR_HOUR         0x02       //  address of HOURS        register
#define  ADDR_DAY          0x03       //  address of DAY OF WK    register
#define  ADDR_STAT         0x03       //  address of STATUS       register
#define  ADDR_DATE         0x04       //  address of DATE         register
#define  ADDR_MNTH         0x05       //  address of MONTH        register
#define  ADDR_YEAR         0x06       //  address of YEAR         register
#define  ADDR_CTRL         0x07       //  address of CONTROL      register
#define  ADDR_CAL          0x08       //  address of CALIB        register
#define  ADDR_ULID         0x09       //  address of UNLOCK ID    register

#define  START_32KHZ  0x80            //  start crystal: ST = b7 (ADDR_SEC)
#define  LP           0x20            //  mask for the leap year bit(MONTH REG)
#define  HOUR_12      0x40            //  12 hours format   (ADDR_HOUR)
#define  PM           0x20            //  post-meridian bit (ADDR_HOUR)
#define  OUT_PIN      0x80            //  = b7 (ADDR_CTRL)
#define  SQWE         0x40            //  SQWE = b6 (ADDR_CTRL)
#define  ALM_NO       0x00            //  no alarm activated        (ADDR_CTRL)
#define  ALM_0        0x10            //  ALARM0 is       activated (ADDR_CTRL)
#define  ALM_1        0x20            //  ALARM1 is       activated (ADDR_CTRL)
#define  ALM_01       0x30            //  both alarms are activated (ADDR_CTRL)
#define  MFP_01H      0x00            //  MFP = SQVAW(01 HERZ)      (ADDR_CTRL)
#define  MFP_04K      0x01            //  MFP = SQVAW(04 KHZ)       (ADDR_CTRL)
#define  MFP_08K      0x02            //  MFP = SQVAW(08 KHZ)       (ADDR_CTRL)
#define  MFP_32K      0x03            //  MFP = SQVAW(32 KHZ)       (ADDR_CTRL)
#define  MFP_64H      0x04            //  MFP = SQVAW(64 HERZ)      (ADDR_CTRL)
#define  ALMx_POL     0x80            //  polarity of MFP on alarm  (ADDR_ALMxCTL)
#define  ALMxC_SEC    0x00            //  ALARM compare on SEC      (ADDR_ALMxCTL)
#define  ALMxC_MIN    0x10            //  ALARM compare on MIN      (ADDR_ALMxCTL)
#define  ALMxC_HR     0x20            //  ALARM compare on HOUR     (ADDR_ALMxCTL)
#define  ALMxC_DAY    0x30            //  ALARM compare on DAY      (ADDR_ALMxCTL)
#define  ALMxC_DAT    0x40            //  ALARM compare on DATE     (ADDR_ALMxCTL)
#define  ALMxC_ALL    0x70            //  ALARM compare on all param(ADDR_ALMxCTL)
#define  ALMx_IF      0x08            //  MASK of the ALARM_IF      (ADDR_ALMxCTL)
#define  OSCON        0x20            //  state of the oscillator(running or not)
#define  VBATEN       0x08            //  enable battery for back-up

/**************************************************Global Variables****************************************************/
unsigned int temp_count;                // general delay counter


/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the RTC.

  Description:
    This routine starts (or restarts) a transfer to/from the RTC, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition

  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(RTC_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(RTC_BUS) );

        if(I2CStart(RTC_BUS) != I2C_SUCCESS)
        {
            DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(RTC_BUS);

    } while ( !(status & I2C_START) );

    return TRUE;
}


/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the RTC.

  Description:
    This transmits one byte to the RTC, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(RTC_BUS));

    // Transmit the byte
    if(I2CSendByte(RTC_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(RTC_BUS));

    return TRUE;
}


/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the RTC.

  Description:
    This routine Stops a transfer to/from the RTC, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(RTC_BUS);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(RTC_BUS);

    } while ( !(status & I2C_STOP) );
}


/***********************************************************************************************************************
*   main
*       Main entry routine
***********************************************************************************************************************/
int main(void)
{
    
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    int                 DataSz;
    UINT32              actualClock;
    BOOL                Success = TRUE;
    UINT8               i2cbyte = 0;
    UINT8               i2cData[10];

    // Set the I2C baudrate
    actualClock = I2CSetFrequency(RTC_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
        DBPRINTF("Error: I2C1 clock frequency (%u) error exceeds 10%%.\n", (unsigned)actualClock);
    }

    // Enable the I2C bus
    I2CEnable(RTC_BUS, TRUE);

    // Initialize the data buffer
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, RTC_ADDRESS, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = ADDR_SEC;                              // RTC register
    i2cData[2] = (0x00|START_32KHZ);                    // Data
    DataSz = 3;

    // Start the transfer to write data to the RTC
    if( !StartTransfer(FALSE) )
    {
        while(1);
    }
    
    // Transmit all data
    Index = 0;
    while( Success && (Index < DataSz) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(RTC_BUS))
            {
                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }

    
    // Initialize the data buffer
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, RTC_ADDRESS, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = ADDR_SEC;              // RTC location to read (high address byte)
    DataSz = 2;
    while(1)
    {
        // Start the transfer to read the RTC.
        if( !StartTransfer(FALSE) )
        {
            while(1);
        }

        // Address the RTC.
        Index = 0;
        while( Success & (Index < DataSz) )
        {
            // Transmit a byte
            if (TransmitOneByte(i2cData[Index]))
            {
                // Advance to the next byte
                Index++;
            }
            else
            {
                Success = FALSE;
            }

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(RTC_BUS))
            {
                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }

        // Restart and send the RTC's internal address to switch to a read transfer

        if(Success)
        {
            // Send a Repeated Started condition
            if( !StartTransfer(TRUE) )
            {
                while(1);
            }

            // Transmit the address with the READ bit set
            I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, RTC_ADDRESS, I2C_READ);
            if (TransmitOneByte(SlaveAddress.byte))
            {
                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(RTC_BUS))
                {
                    DBPRINTF("Error: Sent byte was not acknowledged\n");
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }

        // Read the data from the desired address
        if(Success)
        {
            if(I2CReceiverEnable(RTC_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
            {
                DBPRINTF("Error: I2C Receive Overflow\n");
                Success = FALSE;
            }
            else
            {
                while(!I2CReceivedDataIsAvailable(RTC_BUS));
                i2cbyte = I2CGetByte(RTC_BUS);

            }

        }

        i2cbyte = (i2cbyte & 0x7F);
        // End the transfer (stop here if an error occured)
        StopTransfer();

        if(!Success)
        {
            while(1);
        }
        i2cbyte = 0;
    }

    while(1);
}



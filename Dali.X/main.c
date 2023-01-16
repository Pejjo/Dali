/*
 * File:   main.c
 * Author: pejo
 *
 * Created on den 17 oktober 2022, 13:39
 */


// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 220       // Brown Out Reset Voltage bits (VBOR set to 2.20 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 512      // Watchdog Timer Postscale Select bits (1:512)

// CONFIG3H
#pragma config CCP2MX = PORTB3  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is mulitplexed with RC6)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 2000000


//TX1  RB6
//RX1  RH7
//TX2  RC6
//RX2  RC7
#define LD1  RC2
#define LD2  RC3
#define LD3  RC5
#define PT1  AN0
#define PT2  AN16

#define CAPMODE_INIT    0   //Look for rising edge
#define CAPMODE_SYNC    1   //Find falling edge and calculate width
#define CAPMODE_SAMPL   2   //Sample mid point of succesive bits.
#define CAPMODE_DONE    3   //Full frame received

#define TXMODE_IDLE     0   //Output off
#define TXMODE_SYNC     1   //Sync bit
#define TDMODE_DATA     2   //Send byte data

#define TIMEOUT_TICKS   2450 // ~2.45mS
#define WIDTHMIN_TICKS  300  // ~960 baud
#define WIDTHMAX_TICKS  550 // ~1440 baud
#define RXSAMPLE_TICKS  624 // 3/4 bit @ 1200 baud
#define TXWIDTH_TICKS   417 // ~1200 baud

#define MAX_FLEN        4   // Maximum frame length excpected. Absolute max is 16 (8 bit bitpos counter)

#define UART_RXDONE     1


// Command:
// Data is sent over 9-bit UART @ 38400 Baud.
// First byte with bit 9 set is length bit,[1xxxPxLLL] P = Dali Power, LLL=Length 0-4
// After length byte 0-4 bytes data follows [0DDDDDDDD]
// A byte with length 0 will be responded with Dali Power status if bit P is set.
// Othwerwise, 0 length frames are discarded.
// All length bytes of a response of lager size than 0 will always have bit P set if line has power. 

//--------------------------------
// Functions Declarations
void periodic_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART1_Write(uint8_t);

void dali_InitRx(void);
void dali_StartTx(uint8_t txLen);
uint8_t dali_power(void);

void startUartTx(uint8_t len);
volatile uint8_t Data = 0; // The Data Byte

// Globals
uint8_t statusLed = 0;
volatile uint8_t tick=0;


volatile uint8_t captureMode=CAPMODE_INIT;


volatile uint8_t transmitMode=TXMODE_IDLE;
volatile uint8_t txPos=0;
volatile uint8_t endPos=0;

volatile uint8_t uartStat=0;

uint8_t txBuffer[MAX_FLEN];
uint8_t txLength;

uint8_t rxBuffer[MAX_FLEN];
volatile uint8_t rxLength;

uint8_t txUartBuf[MAX_FLEN + 1];
uint8_t txUartLen;
volatile uint8_t* txUartPtr;

uint8_t rxUartBuf[MAX_FLEN];
volatile uint8_t rxUartLen;


//--------------------------------
// Main Routine
void main(void)
{
    uint8_t transmitting=0;
    OSCCON=(OSCCON & 0x8F) | (0b110 << 4);   
  

    //--[ Peripherals & IO Configurations ]--
  periodic_Init();
  UART1_Init(); // Initialize The UART in Master Mode @ 38400bps
  dali_InitRx();
  //---------------------------

//    TRISC2 = 0; // LED
//    TRISC3 = 0; // LED
//    TRISC5 = 0; // LED

//    LATC2=1; 
//    LATC3=0; 
//    LATC5=1;
  while(1)
  { 
    CLRWDT();
    TRISC3 = 0; // Debug pin
    
    if (uartStat & UART_RXDONE)
    {
         LATC3 = 1;
        if (rxUartLen==0)
        {
            // Command
            if (rxUartBuf[0] == 0x10)
            {
                
                // Read power status
                if (dali_power())
                {
                    txUartBuf[0]=0x10;
                }
                else
                {
                    txUartBuf[0]=0x00;
                }
                startUartTx(1);
            }
        }
        else if (rxUartLen<=4)
        {
            LATC3 = 0;//~LATC3;  // Debug pin
            for (uint8_t fcc=0; fcc < rxUartLen; fcc++)
            {
                txBuffer[fcc]=rxUartBuf[fcc];
            }
            dali_StartTx(rxUartLen);
            transmitting=1;
        }
        uartStat &= ~UART_RXDONE;
    }
    
    // Restart reciever after transmission
    if (transmitting && (transmitMode == TXMODE_IDLE)) //IDLE?
    {
         dali_InitRx();
         transmitting=0;
    }
    if (captureMode==CAPMODE_DONE)
    {
        if (rxLength>0) // RxLenght cant be > MAX_FLEN 
        {
            txUartBuf[0]=rxLength;
            if (dali_power())
            {
                txUartBuf[0] |=0x10;
            }
            for (uint8_t fcc=0; fcc < rxLength; fcc++)
            {
                txUartBuf[fcc+1]=rxBuffer[fcc];
            }
            startUartTx(rxLength + 1);
        }
        dali_InitRx();
    }

  }
 
  return;
}
//--------------------------------
// Functions Definitions
 

void periodic_Init(void)
{
    T2CON = 0x4E;   // 8M / 4 / 16, 1:10 post scale
    PR2 = 125;    // -> 100kHz
    TMR2IE=1;
}

void UART1_Init(void)
{
    ANSELCbits.ANSC7=0;
    // Set The RX-TX Pins to be in UART mode (not io)
    TRISC6 = 1; // As stated in the datasheet
    TRISC7 = 1; // As stated in the datasheet
    BRGH1 = 1; // Set For High-Speed Baud Rate
    BRG161 = 1;  
    SPBRG1 = 51; // 33 = 58.82kbit or 51 = 38400; // Set The Baud Rate To Be 38400 bps
    
    // Enable The Ascynchronous Serial Port
    SYNC1 = 0;
    SPEN1 = 1;
    RX91 = 1; // Enable 9 bit
    //--[ Enable UART Receiving Interrupts ]--
    RC1IE = 1; // UART Receving Interrupt Enable Bit
    PEIE = 1; // Peripherals Interrupt Enable Bit
    GIE = 1; // Global Interrupt Enable Bit
    //------------------
    CREN1 = 1; // Enable Data Continous Reception

   
}

void __interrupt(low_priority) ISR(void)
{ 
    static uint16_t lastEdge;
    static uint16_t width;
    static uint8_t  bitPos;
    static uint8_t  lastLvl;
    uint8_t thisLvl;
    static uint8_t dataByte;
    static uint8_t *dataPtr;
    
    static uint8_t rxUartPos;
    if (RC1IF == 1)
    {
        uint8_t status, data;
        status=RCSTA1;
        data=RCREG1;
        RC1IF = 0; // Not needed
        //LATC3 = ~LATC3;  // Debug pin
        if (status & 0x01) // 9-bit set
        {
            //LATC3 = ~LATC3;  // Debug pin
            rxUartLen=data;
            rxUartPos = 0;
            if (rxUartLen>5)
            {   // We got a command, store data in pos 0.
                rxUartBuf[0] = rxUartLen;
                rxUartLen=0;
            }
        }
        else if (rxUartPos < rxUartLen)
        {
            rxUartBuf[rxUartPos++] = data;
        }
        
        if (rxUartPos == rxUartLen)
        {   
            uartStat|=UART_RXDONE;
            //LATC3 = ~LATC3;  // Debug pin
        }
        
    }
    
    if (TX1IF == 1)
    {
        TX1IF=0;
        if (txUartLen>0)
        {
            txUartLen--;
            txUartPtr++;
            TX9D1=0;
            TXREG1=*txUartPtr;
            TX1IE=1;
        }
        else
        {
            TX1IE=0;
        }
    }
    if ((CCP4IF == 1) && (CCP4IE == 1))
    {
        CCP4IF = 0;
//             LATC3=~LATC3;    
       
        if (captureMode==CAPMODE_INIT)
        {   // First rising edge, sync bit.
            lastEdge=CCPR4;
            LATC3=0;  // Debug pin
            captureMode=CAPMODE_SYNC;
            CCP4CON  = 0x04; // Find falling edge

            rxLength=0;     // Init buffer and length
            dataPtr=rxBuffer;
                
            CCPR5=lastEdge + TIMEOUT_TICKS;
            CCP5IF = 0; // Clear timeout
            CCP5IE = 1; // Enable timeout
                    
        }
        else if (captureMode==CAPMODE_SYNC)
        { // First falling edge. We now can check bit width
            
            width=CCPR4-lastEdge;

            if (( width > WIDTHMIN_TICKS ) && ( width < WIDTHMAX_TICKS))
            {
                captureMode=CAPMODE_SAMPL;
//                LATC3=1;  // Debug pin
                CCP4CON  = 0x05; // Find rising edge
                bitPos = 0; // Reset bit counter

                CCPR1 = CCPR4 + RXSAMPLE_TICKS; // Set sample timer to 3/4 bits
                CCP1CON = 0x0A; // Enable sample interrupt
                CCP1IF = 0; // Important, clear flag for sample timer
                CCP1IE = 1; // Start sample clock          
                
                if ((PORTB & 0x01) != 0x00)
                {   // Check start bit
                    // Invaid level
                    captureMode=CAPMODE_INIT;
                    CCP4CON  = 0x05; // Start with rising edge
                    CCP1IE = 0; // Disable sample timer
                    CCP4IE = 1; // Enable edge
                    CCP5IE = 0; // Disable timeout
                }                
      
            }
            else
            {
                // Invaid Width
                captureMode=CAPMODE_INIT;
                CCP4CON  = 0x05; // Start with rising edge
                CCP1IE = 0; // Disable sample timer
                CCP4IE = 1; // Enable edge
                CCP5IE = 0; // Disable timeout
            }
                 
        }
        else if (captureMode==CAPMODE_SAMPL)
        {
            // Edge is changed in sample interrupt so that we dont trigger on bit egdes, just the mid transition
//            LATC3=~LATC3;    
            if (CCP1IE==0) { // If we sampled previous point
                CCPR1 = CCPR4 + RXSAMPLE_TICKS; // Move sample timer ahead 3/4 bit
                CCP1CON = 0x0A;
                CCP1IF = 0; // Important, clear flag for sample timer
                CCP1IE = 1; // Start sample clock  
            }                
        }
    }
    if ((CCP1IF == 1) && (CCP1IE == 1))
    {   // Sample timer expired. Sample bit and change polarity of edge interrupt
        CCP1IE = 0;
        // Sample interrupt
           if (PORTB & 0x01) 
            {
                // High level, next edge will be falling
                CCP4CON  = 0x04;
            }
            else
            {
                // Low level, next edge will be rising
                CCP4CON  = 0x05;
            }

        LATC3=~LATC3; 
        CCP1IF = 0;
        if (captureMode == CAPMODE_SAMPL)
        {
            CCPR5= CCPR1 + TIMEOUT_TICKS; // Mode timeout ahead
            
            dataByte = (dataByte << 1) & 0xFF;
            dataByte |= (PORTB & 0x01); 
            if ((bitPos & 0x07) == 0x07) // Every byte;
            {
                // LATC3=~LATC3;
                if (rxLength < MAX_FLEN)
                {
                    *dataPtr = dataByte;
                    rxLength ++;
                    dataPtr ++;
                      
                }
                dataByte=0;
            }
            bitPos++;
//            LATC3=~LATC3;
        }
    }
    if ((CCP5IF == 1) && (CCP5IE == 1))
    {
        // Timeout Interrupt
        CCP5IF = 0; // Clear flag
        LATC3=0;  // Debug pin
        if ((bitPos>8)&&((bitPos % 8)==1)) // It was a stopbit, all data (probebly) received
        {
            captureMode=CAPMODE_DONE;
        }
        else
        {
            captureMode=CAPMODE_INIT;
        }
        CCP4CON  = 0x05; // Start with rising edge
        CCP1IE = 0; // Disable sample timer
        CCP4IE = 1; // Enable edge
        CCP5IE = 0; // Disable timeout
    }
    if ((CCP2IF == 1) && (CCP2IE == 1) ) 
    {   // Transmit interrupt
        CCP2IF = 0;
            //
        if (txPos == 0) // First half of sync is sent. Send other half 
        {    
            CCPR2 = CCPR2 + TXWIDTH_TICKS;   
            dataPtr=txBuffer;
            dataByte=*dataPtr;
            txLength--;
            // If nästa byte är en 1:a, enbart flag
            if ((dataByte & 0x80)==0) // 1 -> 1 transition
            {
                LATB3  = 1; // Output on
                CCP2CON = 0x0A; // Generate insterrupt only
                lastLvl = 0; // 10
            }
            else
            {
                CCP2CON = 0x09; // Clear output on match (MARK)
                lastLvl = 1; // 01
            }
        }
        else if (txPos >= endPos)
        {
            transmitMode=TXMODE_IDLE;
            CCP2CON = 0x00; // Release pin
            LATB3  = 1; // Output off
//            LATC3 = 0;  // Debug pin
            CCP2IE = 0;
        }        
        else if ((txPos & 0x01) == 0) // We are in the middle of a bit.
        {
            CCPR2 = CCPR2 + TXWIDTH_TICKS;
            if (((txPos & 0x0F) == 0x00) && txLength)
            {
                dataPtr++;
                dataByte=*dataPtr;
                txLength--;
                
            }
            else
            {
                dataByte = (dataByte << 1) & 0xFF; // First, load next bit value.
            }
            if ((lastLvl == 1) && ((dataByte & 0x80) == 0)) //1 -> 1 transition 
            {
                LATB3  = 1; // Output on
                CCP2CON = 0x0A; // Generate insterrupt only  
                lastLvl = 0;
            }
            else if ((lastLvl == 0) && ((dataByte & 0x80) == 0x80)) // 0 -> 0 transition 
            {
                LATB3  = 0; // Output off
                CCP2CON = 0x0A; // Generate insterrupt only  
                lastLvl = 1;
                
            }
            else if (dataByte & 0x80) // 1 -> 0 transition
            {
                CCP2CON = 0x09; // Clear output on match (MARK)
                lastLvl = 1;
            }
            else // 0 -> 1 transition
            {
                CCP2CON = 0x08; // Set output on match (SPACE)                
                lastLvl = 0;
            }
            
        }
        else if ((txPos & 0x01) == 1) // Start of new bit. We already set this transition, 
        {                             // So our task here is just to set the opposite
                                      // polarity for half-bit transition
            CCPR2 = CCPR2 + TXWIDTH_TICKS;

            if (lastLvl == 1)// 0 -> 1 transition 
            {
                CCP2CON = 0x08; // Set output on match (SPACE)
                
            }
            else // 1 -> 0 transition
            {
                CCP2CON = 0x09; // Clear output on match (MARK)
                
            }
        }       

        txPos++;
    }
 
    if (TMR2IF == 1)
    {
        tick++;
        TMR2IF = 0; // Clear The Flag
    }

 }

void dali_InitRx(void)
{
    while (transmitMode!=TXMODE_IDLE); // Wait for any ongoing transmissions to end 
    captureMode=CAPMODE_INIT;
    rxLength=0;
    TRISC3 = 0; // Debug pin out
    TRISB0 = 1; // Input capture........
    LATB3  = 1; // Output off
    TRISB3 = 0; // Output
    
    T1CON = 0x13; // Fosc/4/2=1M, enabled and 16 bit
    CCP1CON = 0x0A;  //Interrupt on timeout
    CCP2CON = 0x00;  // Diable transmitter
    CCP4CON  = 0x05; // Start with rising edge
    CCP5CON = 0x0A;  //Interrupt on timeout
    CCPTMRS0 = 0x00; // CCP1 -> Timer1, CCP2 -> Timer1
    CCPTMRS1 = 0x00; // CCP4 -> Timer1, CCP5 -> Timer1
    CCP1IE = 0; // Disable sample timer
    CCP2IE = 0; // Disable transmitter
    CCP4IE = 1; // Enable edge
    CCP5IE = 0; // Disable timeout
}

void dali_StartTx(uint8_t txLen)
{
    while ((captureMode!=CAPMODE_INIT) && (captureMode!=CAPMODE_DONE)); // Wait for any ongoing reception to complete
    transmitMode=TXMODE_SYNC;
    txPos = 0;
    endPos = ((txLen << 4) + 1) & 0xFF ; // * 2 * 8 + 1 bits
    txLength = txLen;
    CCP2CON = 0x08; // Set output on match
    CCPR2 = TMR1 + TXWIDTH_TICKS;
//    LATC3 = 1;  // Debug pin
    LATB3  = 0; // Output on
    CCP1IE = 0; // Disable sample timer
    CCP2IE = 1; // Enable transmitter
    CCP4IE = 0; // Disable edge
    CCP5IE = 0; // Disable timeout
}

uint8_t dali_power(void)
{
    return (((PORTB & (1<<5)) == 0));
}


//--------------------------------
 
void UART1_Write(uint8_t data)
{
  while(!TX1IF);
  TXREG1 = data;
}

void startUartTx(uint8_t len)
{
    while (TRMT1 == 0); // Ongoing transmission
    if (len > 0)
    {
        txUartLen = len - 1;
        txUartPtr = txUartBuf;
        TX91=1;
        TXEN1=1;
        TX9D1=1;
        TXREG1=*txUartPtr;
        TX1IE=1;
    }
}

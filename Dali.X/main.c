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
#define WIDTHMIN_TICKS  333  // ~960 baud
#define WIDTHMAX_TICKS  500 // ~1440 baud
#define TXWIDTH_TICKS   417 // ~1200 baud

#define MAX_FLEN        4   // Maximum frame length excpected. Absolute max is 16 (8 bit bitpos counter)


//--------------------------------
// Functions Declarations
void periodic_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART1_Write(uint8_t);

void dali_InitRx(void);
void dali_StartTx(uint8_t *txPtr, uint8_t txLen);
uint8_t dali_power(void);
void sendStr(uint8_t *data, uint8_t len);
volatile uint8_t Data = 0; // The Data Byte

// Globals
uint8_t statusLed = 0;
volatile uint8_t tick=0;


volatile uint8_t captureMode=CAPMODE_INIT;


volatile uint8_t transmitMode=TXMODE_IDLE;
volatile uint8_t txPos=0;
volatile uint8_t endPos=0;

uint8_t txBuffer[MAX_FLEN];
uint8_t txLength;


uint8_t rxBuffer[MAX_FLEN];
uint8_t rxLength;

//--------------------------------
// Main Routine
void main(void)
{
    OSCCON=(OSCCON & 0x8F) | (0b110 << 4);   
  

    //--[ Peripherals & IO Configurations ]--
  periodic_Init();
  UART1_Init(); // Initialize The UART in Master Mode @ 38400bps
  dali_InitRx();
//  UART2_Init(); // Initialize The UART in Master Mode @ 38400bps
//  analog_Init();
  //---------------------------

//    TRISC2 = 0; // LED
//    TRISC3 = 0; // LED
//    TRISC5 = 0; // LED

//    LATC2=1; 
//    LATC3=0; 
//    LATC5=1;
  sendStr("Boot",4);
  while(1)
  { 
    CLRWDT();
    if (tick==100)
    {
        statusLed=1;
        if (dali_power())
        {
            UART1_Write('1');
            tick=0;
//            LATC3 = 1;
        }
        else
        {
            UART1_Write('0');
            tick=0;
            txBuffer[0]=0x53;
            txBuffer[1]=0x46;
            txBuffer[2]=0xF1;
            txBuffer[3]=0x3C;
            dali_StartTx(txBuffer,2);
            UART1_Write('T');
//            LATC3 = 0;
        }
        

         
    }
    if (Data==0x30)
    {
//        txBuffer[0]=0xA5;
//        dali_StartTx(txBuffer,1);
//        UART1_Write('T');
//        Data=0;
    }
   if ((captureMode==CAPMODE_DONE) && (transmitMode == TXMODE_IDLE)) 
   {
       
       UART1_Write('C');
       UART1_Write(rxLength+0x30);
       UART1_Write(rxBuffer[0]);
       UART1_Write(rxBuffer[1]);
       UART1_Write(rxBuffer[2]);
       UART1_Write(rxBuffer[3]);       
       UART1_Write(';');
       dali_InitRx();
   }
//    PORTC = 0x24;
    // Stay Idle, Everything is handled in the ISR !
  }
  return;
  //---------------------------
  while(1)
  {
      __delay_ms(250);
      UART1_Write(Data);
      __delay_ms(250);
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

    // Set The RX-TX Pins to be in UART mode (not io)
    TRISC6 = 1; // As stated in the datasheet
    TRISC7 = 1; // As stated in the datasheet
    BRGH1 = 1; // Set For High-Speed Baud Rate
    BRG161 = 1;  
    SPBRG1 = 51; // 33 = 58.82kbit or 51 = 38400; // Set The Baud Rate To Be 38400 bps
    
    // Enable The Ascynchronous Serial Port
    SYNC1 = 0;
    SPEN1 = 1;

    //--[ Enable UART Receiving Interrupts ]--
    RC1IE = 1; // UART Receving Interrupt Enable Bit
    PEIE = 1; // Peripherals Interrupt Enable Bit
    GIE = 1; // Global Interrupt Enable Bit
    //------------------
    CREN1 = 1; // Enable Data Continous Reception

    TXEN1 = 1; // Enable UART Transmission
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
    if (RC1IF == 1)
    {
//        uint8_t UART1_Buffer;
        Data = RC1REG; // Read The Received Data Buffer
        RC1IF = 0; // Clear The Flag
//        UART2_Write(UART1_Buffer);
    }
    if ((CCP4IF == 1) && (CCP4IE == 1))
    {
        CCP4IF = 0;
        
        if (captureMode==CAPMODE_INIT)
        {   // First rising edge, sync bit.
            lastEdge=CCPR4;
            captureMode=CAPMODE_SYNC;
            CCP4CON  = 0x04; // Find falling edge
            CCPR5=lastEdge + TIMEOUT_TICKS;
            CCP5IE = 1; // Enable timeout
            //LATC3=1;  // Debug pin
            
        }
        else if (captureMode==CAPMODE_SYNC)
        {
            width=CCPR4-lastEdge;
            if (( width > WIDTHMIN_TICKS ) && ( width < WIDTHMAX_TICKS))
            {
                captureMode=CAPMODE_SAMPL;
                CCPR1 = CCPR4 + (width >> 1); // Add half width delay on fist sample
                CCP1IE = 1; // Start sample clock
                CCP4IE = 0; // Stop edge
                bitPos = 0;
                //LATC3=0;  // Debug pin
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
    }
    if ((CCP1IF == 1) && (CCP1IE == 1))
    {
        // Sample interrupt
        CCP1IF = 0;
        if (captureMode == CAPMODE_SAMPL)
        {
            //LATC3=~LATC3; // Toggle debug pin
            CCPR1 = CCPR1 + width; // Move ahead to next sample point
            CCPR5= CCPR4 + TIMEOUT_TICKS; // Mode timeout ahead
            
            thisLvl=(PORTB & 0x01);
            
            if (bitPos == 0) // First transition after start half-bit,
            {
                if (thisLvl != 0x00)
                {
                    // Invaid level
                    captureMode=CAPMODE_INIT;
                    CCP4CON  = 0x05; // Start with rising edge
                    CCP1IE = 0; // Disable sample timer
                    CCP4IE = 1; // Enable edge
                    CCP5IE = 0; // Disable timeout
                }
                else
                {
                    dataByte=0;
                    dataPtr=rxBuffer;
                    rxLength=0;
                }
            }
            else if ((bitPos & 0x01) == 0) // Data every even bitpos (two tansitions/bit)
            {
                if ((lastLvl == 0) && (thisLvl == 1))
                {
                    dataByte = (dataByte << 1) & 0xFF;
                    //dataByte |= 0x00; Not needed to waste a cycle                     
                }
                else if ((lastLvl == 1) && (thisLvl ==0))
                {
                    dataByte = (dataByte << 1) & 0xFF;
                    dataByte |= 0x01; 
                }
                else 
                {
                    captureMode=CAPMODE_DONE; 
                    // Stopp after two 00 or 11.
                }
                if ((bitPos & 0x0F) == 0) // Every byte;
                {
                    if (rxLength < MAX_FLEN)
                    {
                        *dataPtr = dataByte;
                        rxLength ++;
                        dataPtr ++;
                    }
                    dataByte=0;
                }
            }
            bitPos++;
            lastLvl=thisLvl;
        }
    }
    if ((CCP5IF == 1) && (CCP5IE == 1))
    {
        // Timeout Interrupt
        CCP5IF = 0; // Clear flag
        //LATC3=0;  // Debug pin
        captureMode=CAPMODE_INIT;
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
            LATC3 = 0;  // Debug pin
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
                LATC3 = ~LATC3;  // Debug pin
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

void dali_StartTx(uint8_t *txPtr, uint8_t txLen)
{
    while ((captureMode!=CAPMODE_INIT) && (captureMode!=CAPMODE_DONE)); // Wait for any ongoing reception to complete
    transmitMode=TXMODE_SYNC;
    txPos = 0;
    endPos = ((txLen << 4) + 1) & 0xFF ; // * 2 * 8 + 1 bits
    txLength = txLen;
    CCP2CON = 0x08; // Set output on match
    CCPR2 = TMR1 + TXWIDTH_TICKS;
    LATC3 = 1;  // Debug pin
    LATB3  = 0; // Output on
    CCP1IE = 0; // Disable sample timer
    CCP2IE = 1; // Enable transmitter
    CCP4IE = 0; // Disable edge
    CCP5IE = 0; // Disable timeout
}

uint8_t dali_power(void)
{
    return (((PORTB & (1<<5)) == (1<<5)));
}


//--------------------------------
 
void UART1_Write(uint8_t data)
{
  while(!TX1IF);
  TXREG1 = data;
}

void sendStr(uint8_t *data, uint8_t len)
{
    while(--len)
        UART1_Write(*(data++));
}
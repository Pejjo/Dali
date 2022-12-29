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
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
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

#define _XTAL_FREQ 4000000


//TX1  RB6
//RX1  RH7
//TX2  RC6
//RX2  RC7
#define LD1  RC2
#define LD2  RC3
#define LD3  RC5
#define PT1  AN0
#define PT2  AN16


//--------------------------------
// Functions Declarations
void periodic_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART1_Write(uint8_t);
void UART2_Write(uint8_t);
void analog_Init(void);
uint8_t analog_Done(void);
void analog_Start(uint8_t ch);
uint8_t analog_Get(void);


uint8_t UART_Read(void);
// Globals
uint8_t statusLed = 0;
volatile uint8_t tick=0;
//--------------------------------
// Main Routine
void main(void)
{
    OSCCON=(OSCCON & 0x8F) | (0b110 << 4);   
  uint8_t Data = 0; // The Data Byte

    //--[ Peripherals & IO Configurations ]--
  periodic_Init();
  UART1_Init(); // Initialize The UART in Master Mode @ 38400bps
  UART2_Init(); // Initialize The UART in Master Mode @ 38400bps
  analog_Init();
  //---------------------------

    TRISC2 = 0; // LED
    TRISC3 = 0; // LED
    TRISC5 = 0; // LED

    LATC2=1; 
    LATC3=0; 
    LATC5=1; 
  while(1)
  { 
    CLRWDT();
    if (tick==100)
    {
        statusLed=1;
        if (analog_Done())
        {
            uint8_t adv=analog_Get();
            analog_Start(16);
            UART2_Write(adv);
            tick=0;
        }
         
    }
      if (TXSTA2 & 0x02)
      {
        // Disable driver enable for RS485, (crappy PICs doent have neither hw nor interrupt for this))
        LATB0=0;
        LATB3=0;
        CREN2 = 1; // Reenable Data Continous Reception
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
 
void UART2_Init(void)
{

    // Set The RX-TX Pins to be in UART mode (not io)
    TRISB0 = 0; // TE
    TRISB3 = 0; // RE
    TRISB6 = 1; // As stated in the datasheet
    TRISB7 = 1; // As stated in the datasheet
    BRGH2 = 1; // Set For High-Speed Baud Rate
    BRG162 = 1;  
    SPBRG2 = 51; // Set The Baud Rate To Be 38400 bps
    
    // Enable The Ascynchronous Serial Port
    SYNC2 = 0;
    SPEN2 = 1;

    //--[ Enable UART Receiving Interrupts ]--
    RC2IE = 1; // UART Receving Interrupt Enable Bit
    PEIE = 1; // Peripherals Interrupt Enable Bit
    GIE = 1; // Global Interrupt Enable Bit
    //------------------
    CREN2 = 1; // Enable Data Continous Reception

    TXEN2 = 1; // Enable UART Transmission
}

void __interrupt(low_priority) ISR(void)
{ 
    if (RC1IF == 1)
    {
        uint8_t UART1_Buffer;
        UART1_Buffer = RC1REG; // Read The Received Data Buffer
        RC1IF = 0; // Clear The Flag
        UART2_Write(UART1_Buffer);
    }
    if (RC2IF == 1)
    {
        uint8_t UART2_Buffer;
        UART2_Buffer = RC2REG; // Read The Received Data Buffer
        RC2IF = 0; // Clear The Flag
        UART1_Write(UART2_Buffer);
        LATC3 = 1; // Flash green
    }
    if (TMR2IF == 1)
    {
        tick++;
        TMR2IF = 0; // Clear The Flag
        LATC2=1;  // Clear TX-led
        if (statusLed)
        {
            LATC3 = 0; // Re-lite status if we should
        }
    }
}

void analog_Init(void)
{
    ANSELA=0x01;    // An0
    ANSELC=0x10;    // An16
    TRISA0 = 1; // As stated in the datasheet
    TRISC4 = 1; // As stated in the datasheet
    ADCON1 = 0x00; // VDD-VSS vref
    ADCON2 = 0x21; // Left, 8ADT Fosc/8
    ADCON0 = 0x03;
}

uint8_t analog_Done(void)
{
    return !(ADCON0 & 0x02);
}

void analog_Start(uint8_t ch)
{
    ADCON0 = (ch << 2) | 0x03u;
}

uint8_t analog_Get(void)
{
    return ADRESH;
}


//--------------------------------
 
void UART1_Write(uint8_t data)
{
  while(!TX1IF);
  TXREG1 = data;
}

void UART2_Write(uint8_t data)
{
    LATC2=0;
    LATB0=1;
    CREN2 = 0; // Disable Data Reception, since disableing tanciever not is workning due to lack of pullup
    while(!TX2IF);
    TXREG2 = data;
}


/*
 * File:   main.c
 * Author: Yohannes Andegergsh
 * Projekt: TIMER0
 * Created on 22. Januar 2021, 15:54
 */
// PIC16F1787 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "PBA_config.h"
//#include "LCD_2x16.h"

#define _XTAL_FREQ 8000000    // Frequenz 8MHz

#define SUCESS (bool) 0x00;
#define n_SUCESS (bool) 0x01;

typedef struct TIMER0_config
{
    bool    weak_pull_up;                  //Weak Pull-Up Enable bit.
    bool    interrupt_edge;                   //Interrupt Edge Select bit.
    bool    timer0_source_clock;          //Timer0 source clock select bit.
    bool    timer0_source_edage;              //Timer0 Scource Edge Select bit.
    bool    prescaler_ass_pin;          //Prescaler assignment bit.
    uint8_t prescaler_select_bits;      //prescaler Rate Select bits.
}TIMER0_config_t;

/* ADC module Configuration 
 * @ADC module abschalten
 * @Positive and Negative Channel section
 * @Positive and Negative Voltage reference
 * @Clock conversion
 * @retrun Ture if ADC is on else false
 */

static void TIMER0_Init(TIMER0_config_t* config)
{
    assert(config != NULL);
    
    //Assert funktion prüft, ob der eingegebne Wert gülrig ist. Mann kann es einfach debuggen
    assert(config->prescaler_select_bits <= (uint8_t)0x07 && config->prescaler_select_bits >= (uint8_t)0x00);
    OPTION_REGbits.PS = config->prescaler_select_bits;  //prescaler Rate Select bits.
    OPTION_REGbits.PSA = config->prescaler_ass_pin;      //Prescaler assignment bit.
    OPTION_REGbits.T0SE = config->timer0_source_edage;    //Timer0 Scource Edge Select bit.
    OPTION_REGbits.TMR0CS = config->timer0_source_clock;    //Timer0 source clock select bit.
    OPTION_REGbits.INTEDG = config->interrupt_edge;         //Interrupt Edge Select bit.
    OPTION_REGbits.nWPUEN = config->weak_pull_up;           //Weak Pull-Up Enable bit.
	
	//Config Interrupt for TIMER0
    INTCONbits.GIE = true;      //Enable Interrpt-On-Change 
    INTCONbits.TMR0IF = false;
    INTCONbits.TMR0IE = true;
    
   // return SUCESS;  //if Configuration was sucess retrun a value null
    
}
//*****************************************************************************************************

//***********PORTs Initialisierung****************
static void PORTs_Initialisierung(void)
{
    // *** PORT CONFIG ***
    ANSELD = ANSELE = 0x00;  //Digital = 0; Analog = 1
    TRISD  = TRISE = 0x00;  //OUTPUT = 0; INPUT = 1;
    
    //****PORT CONFIG FOR LCD*****
    ANSELAbits.ANSA2 = ANSELAbits.ANSA4 = ANSELAbits.ANSA5 = ANSELEbits.ANSE2 = false; //As Digital
    TRISAbits.TRISA2 = TRISAbits.TRISA4 = TRISAbits.TRISA5 = false; //As output
    TRISCbits.TRISC0 = TRISCbits.TRISC1 = TRISCbits.TRISC2 = TRISEbits.TRISE2 = false; //As output
    //*****LCD PORT CONFIG END********
    
    //ANSELB = 0x02; TRISB = 0x02; //Aufsatzprint AN10 Analog-Digital
    ANSELAbits.ANSA0 = 1; TRISAbits.TRISA0 = 1;
    ANSELAbits.ANSA1 = 1; TRISAbits.TRISA1 = 1;
    
    //ANSELB = 0x02; TRISB = 0x02; //Aufsatzprint PORTB 0
    ANSELBbits.ANSB0 = 1; TRISBbits.TRISB0 = 1;
    
    LATD = LATEbits.LATE0 = LATEbits.LATE1 = 0x00;  //PORT D&E auf 0 setzen
}

static void __interrupt() IRS(void)
{
    if(INTCONbits.TMR0IE & INTCONbits.TMR0IF)
    {
        /*
         * 195* 128 Microsekunden = 24.96 Milliesekunden
         * 255 - 195 = 60 also offset ist 60
         * */
        TMR0 = 60; 
        LATD ^= 0xFF;
        INTCONbits.TMR0IF = false; //Clear the flag
    }
}

//Look datasheet to configuration the TIMER0
TIMER0_config_t timer_config = {
    0x00,   //don't care 
    0x00,   //don't care 
    0x00,   //interval instruction dycle clock(Fosc/4)
    0x00,   //don't care
    0x00,   //When the bit is cleared Prescaler is asssigned to the Timer0 module
    0x07,   // (Fosc/4) / 7 also pro Schritte 128 Microsekunden.
    
}; //With that configuration I want intterupt every 1ms

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    LCD_Init(V_3V3);
    TIMER0_Init(&timer_config);
    
    
    for(;;)
    {
       
      
              
    }
}

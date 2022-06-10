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
//#include "PBA_config.h"

#define _XTAL_FREQ 8000000    // Frequenz 8MHz

#define SUCESS (bool) 0x00
#define n_SUCESS (bool) 0x01
#define Disable (bool) 0x00
#define Enable  (bool) 0x01


//Global variables


typedef struct TIMER2_config
{
    bool    timer2_interrupt_bits;  //Enable/Disable Timer2 Interrupt.
    uint8_t postscaler_select_bits; //Timer2 Output Postscler Select bits.
    bool    timer2_on_off_bits;     //Tiemr2 On bit.
    uint8_t prescaler_select_bits;  //Timer2 Clock Prescale Select bits.
    uint8_t PR2_register;           //Load the register;
}TIMER2_config_t;

/* TIMER2 module Configuration 
 * @Interrpt    Enable/Disalbe
 * @Postscaler  select bits
 * @Timer2      On/Off the timer2
 * @Prescaler   select bits
 */
static void TIMER2_Init(TIMER2_config_t* config)
{
    assert(config != NULL);
    
    //Assert funktion prüft, ob der eingegebne Wert gülrig ist. Mann kann es einfach debuggen
    assert((config->prescaler_select_bits <= (uint8_t)0x03) && (config->prescaler_select_bits >= (uint8_t)0x00));
    assert((config->postscaler_select_bits <= (uint8_t)0x0F) && (config->postscaler_select_bits >= (uint8_t)0x00));
    T2CON = (config->postscaler_select_bits <<3) | (config->timer2_on_off_bits <<2) | (config->prescaler_select_bits <<0);
    PR2 = config->PR2_register; //load the register PR2. Interrupt on TRM2 mach with PR2 
    
    if(config->timer2_interrupt_bits == true)
    {
        //Config Interrupt for TIMER0
        INTCONbits.GIE = true;      //Enable Global Interrupt
        INTCONbits.PEIE = true;     //Peripheral interrupt Ebable bit
        PIR1bits.TMR2IF = false;    //Clear the interrupt flag before Timer2 interrupt enabled.
        PIE1bits.TMR2IE = true;     //Enable the Timer2 interrupt.
    }
    
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
    if(PIE1bits.TMR2IE & PIR1bits.TMR2IF)
    {
        LATD ^= 0xFF;
        PIR1bits.TMR2IF = false; //Clear the flag
    }
}

//Look datasheet Page 212 to configuration the TIMER2
TIMER2_config_t timer_config = {
    
    Enable, //Enable interrupt
    0x0F,   //Set postscaler "delay times 1,2,3...16"
    Enable, //Enable Timer2
    0x03,   //Set prescaler "00 Cl/1, 01, Cl/4, Cl/16, Cl/64".
    195,    //load to the PR2 register
};

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    //LCD_Init(V_3V3);
    TIMER2_Init(&timer_config);
    
    
    for(;;)
    {
       
        
              
    }
}

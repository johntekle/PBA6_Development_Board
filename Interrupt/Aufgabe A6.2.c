/*
 * File:   main.c
 * Author: Yohannes Andegergsh
 * Projekt: ADC Wandler
 * Created on 30. Oktober 2020, 15:54
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

#define _XTAL_FREQ 8000000    // Frequenz 8MHz

uint8_t counter = 0;


//Globale Variable

//PORTs initalisierung
void PORTs_Initialisierung(void)
{
    //****PORT CONFIG FOR LCD*****
    //ANSELAbits.ANSA2 = ANSELAbits.ANSA4 = ANSELAbits.ANSA5 = ANSELEbits.ANSE2 = false; //As Digital
    //TRISAbits.TRISA2 = TRISAbits.TRISA4 = TRISAbits.TRISA5 = false; //As output
    //TRISCbits.TRISC0 = TRISCbits.TRISC1 = TRISCbits.TRISC2 = TRISEbits.TRISE2 = false; //As output
    //*****LCD PORT CONFIG END********
    
    //Digital Eingang
    ANSELBbits.ANSB0 = ANSELBbits.ANSB1 = ANSELBbits.ANSB2 =false;     // Pin als Digital
    TRISBbits.TRISB0 = TRISBbits.TRISB1 = TRISBbits.TRISB2 = true;    // Pin als Eingang
    
    //Für PORTD LEDs Anzeige
    ANSELD = false;
    TRISD  = false;
    
    LATD = 0x00; //zürucksetzen
}

void __interrupt() ISR(void)
{
    __delay_ms(10);
    if(INTCONbits.INTE & INTCONbits.INTF)
    {
        if(PORTBbits.RB0)
            counter++;
        if(counter > 2)
            counter = 0;
        
        INTCONbits.INTF = false;    //Clear External Flag bit
        
    }
}

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    INTCONbits.GIE  = true;     //Enable Global Interrupt
    OPTION_REGbits.INTEDG = true;   //Interrupt rising edge 
    INTCONbits.INTF = false;    //Clear External Flag bit
    INTCONbits.INTE = true;     //Enable External Interrpt 
    
    
    for(;;)
    {
        static uint8_t i = 0;
        
        switch(counter)
        {
            case 0:
                LATD = (uint8_t)0x00;
                i = 0;
                break;
            case 1:
                LATD = i++;
                __delay_ms(1000);
            default:
                break;
        }
            
    }
}

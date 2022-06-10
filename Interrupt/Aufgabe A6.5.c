/*
 * File:   main.c
 * Author: Yohannes Andegergsh
 * Projekt: Aufgabe 6.4
 * Created on xx. xx 20xx, xx:xx
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

#define _XTAL_FREQ 8000000    // Frequenz 8MHz

uint8_t counter = 0;


//Globale Variable

//PORTs initalisierung
void PORTs_Initialisierung(void)
{
    //****PORT CONFIG FOR LCD*****
    ANSELAbits.ANSA2 = ANSELAbits.ANSA4 = ANSELAbits.ANSA5 = ANSELEbits.ANSE2 = false; //As Digital
    TRISAbits.TRISA2 = TRISAbits.TRISA4 = TRISAbits.TRISA5 = false; //As output
    TRISCbits.TRISC0 = TRISCbits.TRISC1 = TRISCbits.TRISC2 = TRISEbits.TRISE2 = false; //As output
    //*****LCD PORT CONFIG END********
    
    //Digital Eingang
    ANSELBbits.ANSB0 = ANSELBbits.ANSB1 = ANSELBbits.ANSB5 =false;     // Pin als Digital
    TRISBbits.TRISB0 = TRISBbits.TRISB1 = TRISBbits.TRISB5 = true;    // Pin als Eingang
    
    //Für PORTD LEDs Anzeige
    ANSELD = false;
    TRISD  = false;
    
    LATD = 0x00; //zürucksetzen
}

bool state = 0;

void __interrupt() ISR(void)
{
    __delay_ms(10);
    if(INTCONbits.IOCIE & IOCBFbits.IOCBF5){
        if(PORTBbits.RB5) counter++;
        if(counter > 2) counter = 0;
        IOCBFbits.IOCBF5 = false;    //Clear     INTCONbits.IOCIE = true;     //Enable Interrpt-On-Change Flag bit
    }
    
    if(INTCONbits.IOCIE & IOCBFbits.IOCBF1){
        if(PORTBbits.RB1) state = !state;        
        IOCBFbits.IOCBF1 = false;    //Clear     INTCONbits.IOCIE = true;     //Enable Interrpt-On-Change Flag bit
    }
}

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    INTCONbits.GIE  = true;     //Enable Global Interrupt
    IOCBPbits.IOCBP5 = true;   //Interrupt rising edge 
    IOCBPbits.IOCBP1 = true;   //Interrupt rising edge  
    IOCBFbits.IOCBF5 = false;    //Clear External Flag bit
    IOCBFbits.IOCBF1= false;    //Clear External Flag bit
    INTCONbits.IOCIE = true;     //Enable Interrpt-On-Change
    
    //Anzeige Initialisierung
    LCD_Init(V_3V3);
    
    
    for(;;)
    {
        static uint8_t z_sek = 0, sek = 0, min = 0; //Variable nur einmal definieren
        z_sek++; //Auf jeder 1/10 Sekunden inkrementieren
        __delay_ms(100); //delay 100 Millisekunden
        
        if(10 == z_sek){
            z_sek = 0; sek++; 
        }
        if(59 == sek){
            sek = 0; min++;  
        }
        if(59 == min){
            z_sek = sek = min = 0;
        }
            
            
        
        if(!state)
        {
            switch(counter)
            {
                case 0:
                    LCD_Clear(); printf("00:00.0"); //Anzeige löschen und auf default setzen
                    z_sek = sek = min = 0;  //Alle Variable zurücksetzen
                    break;
                case 1:
                    LCD_Home(); printf("%2i:%2i.%i",min,sek,z_sek);
                    //Wenn Sekunden kleiner als 9 füge 0
                    if(10 > sek){
                       LCD_GotoXY(3,0);  LCD_Putc('0'); //Kordination für Sek XY
                    }
                    //Wenn Minuten kleiner als 9 füge 0
                    if(10 > min){
                       LCD_GotoXY(0,0); LCD_Putc('0');  //Kordination für min XY
                    }
                    break;
                default:
                    break;
            } 
        }
        
            
    }
}

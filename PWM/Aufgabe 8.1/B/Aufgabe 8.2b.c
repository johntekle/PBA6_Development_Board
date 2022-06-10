/*
 * File:   main.c
 * Author: Yohannes Andegergsh
 * Projekt: PWM Aufgabe 8.1b
 * Created on 23. April 2021, 15:00
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

#define Disable (bool)false
#define Enable  (bool)true

//*************************************TMR2**************************************************************

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
    PR2 = config->PR2_register; //load the register PR2. Interrupt on TRM2 mach with PR2
    T2CON = (uint8_t)((config->postscaler_select_bits <<3) | (config->timer2_on_off_bits <<2) | (config->prescaler_select_bits <<0)); 
    
    if(config->timer2_interrupt_bits == true)
    {
        //Config Interrupt for TIMER0
        INTCONbits.GIE = true;      //Enable Global Interrupt
        INTCONbits.PEIE = true;     //Peripheral interrupt Ebable bit
        PIR1bits.TMR2IF = false;    //Clear the interrupt flag before Timer2 interrupt enabled.
        PIE1bits.TMR2IE = true;     //Enable the Timer2 interrupt.
    }
    
}

//Look datasheet Page 212 to configuration the TIMER2
TIMER2_config_t timer2_config = {
    
    Disable, //Enable interrupt
    0x00,   //Set postscaler "delay times 1,2,3...16" is 16
    Enable, //Enable Timer2
    0x02,   //Set prescaler "00 Cl/1, 01, Cl/4,10 Cl/16,11 Cl/64".
    250,    //(1/500Hz)/(1/(2Mhz/16)) = 250
};

//***********PWM Configuration****************
void PWM_Config(void)
{
    APFCONbits.CCP1SEL = false; //Select ouput pin for PWM (Pin C2)
    TRISC2 = true; //Disable the CCPx pin output driver by setting the associated TRIS bit.
    CCP1CONbits.CCP1M = (uint8_t)0xFF; //Configure the CCP module for PWM mode
    CCPR1L = 10; //PWM duty cycle value. Nicht relevant
    TIMER2_Init(&timer2_config); //Call the Timer 2 Config Function
    while(!PIR1bits.TMR2IF); //Wait until the Timer overflows
    TRISC2 = false; //Enable the CCPx pin output driver by clearing the associated TRIS bit.
    
}

static void __interrupt() ISR(void)
{
    
}

//***********PORTs Initialisierung****************
static void PORTs_Initialisierung(void)
{
    
    //****PORT CONFIG FOR LCD*****
//    ANSELAbits.ANSA2 = ANSELAbits.ANSA4 = ANSELAbits.ANSA5 = ANSELEbits.ANSE2 = false; //As Digital
//    TRISAbits.TRISA2 = TRISAbits.TRISA4 = TRISAbits.TRISA5 = false; //As output
//    TRISCbits.TRISC0 = TRISCbits.TRISC1 = TRISCbits.TRISC2 = TRISEbits.TRISE2 = false; //As output
    //*****LCD PORT CONFIG END********
    
    //Config Analog pin
    ANSELBbits.ANSB5 = true; TRISBbits.TRISB5 = true;
    
    //Pin B1 as Digital input
    ANSELBbits.ANSB1 = false; TRISBbits.TRISB1 = true;
    
    //Pins as Digital output
    ANSELBbits.ANSB2 = false; TRISBbits.TRISB2 = false;
    ANSELBbits.ANSB3 = false; TRISBbits.TRISB3 = false;
    
    //Pin zurucksetzen
    LATBbits.LATB2 = LATBbits.LATB3 = false; //Pins auf null setzen gemäss Aufgabe
}

/*********************************************************************
 * Function:        unsigned char negflank(void)
 ********************************************************************/
//uint8_t negflank(void) 
//{
//    static unsigned char old_State; //Alte zustand des PORTC speichern
//    unsigned char neg_Flank = 0;    //negative Flanke speichern
//    unsigned char new_State = 0;    //neue zustand des PORTC speichern
//    
//    new_State = PORTB;
//    neg_Flank = ~new_State & old_State;
//    old_State = new_State;
//    
//    return neg_Flank; //Erkannte Flanke zurück geben.
//
//}

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    PWM_Config();
    //Endloss Schleife
    for(;;)
    {
        static bool state = false; //Um der Register nicht mehrers mal zuweisen.
        
        //10% Duty Cycle
         if(PORTBbits.RB1 & state)
         {
             CCPR1L = 25;
             state = false;
         }
        //20% Duty Cycle
         else if(!PORTBbits.RB1 & !state)
         {
            CCPR1L = 50;
            state = true;
         }
             
        
    }
}

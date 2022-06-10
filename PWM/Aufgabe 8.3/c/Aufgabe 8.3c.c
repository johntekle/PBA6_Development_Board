/*
 * File:   main.c
 * Author: Yohannes Andegergsh
 * Projekt: PWM Aufgabe 8.2
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
#define PWM_Duty(x) CCPR1L = (uint8_t)((x >> 2) & (uint8_t)0xFF); CCP1CONbits.DC1B = (uint8_t)(x & (uint8_t)0x03)

//ADC wurde hier defineirt
#define Analog_to_Digital_converter_10bit
//#define Analog_to_Digital_converter_12bit


//************************************ADC**************************************************************
//Wenn es Analog_to_Digital_converter definiert ist, soll die ADCs-Funktionen sichtbar sein.
#if defined Analog_to_Digital_converter_10bit || defined Analog_to_Digital_converter_12bit

//#define ADC_INTERRUPT
#define ADC_POLLING

#define ADC_OFF false
#define ADC_ON  true

static uint16_t ADC_result = 0;
static uint8_t CH_SEL = 0;

typedef struct FVR_config
{
    bool enable_fixed_voltage_reference;
    uint8_t ADC_fixed_voltage_reference;
}FVR_config_t;

typedef struct ADC_config
{
    bool    ADC_Enable;                   //Enabe the ADC module
    uint8_t positive_channel_select;      //Select positive channel
    uint8_t negative_channel_select;      //Select negative channel
    uint8_t positive_voltage_reference;   //Configuration positive voltage reference
    bool    negative_voltage_reference;   //Configuration negarive volltage referece
    uint8_t conversion_clock;             //The source of the conversion clock.
}ADC_config_t;

/* ADC module Configuration 
 * @ADC module abschalten
 * @Positive and Negative Channel section
 * @Positive and Negative Voltage reference
 * @Clock conversion
 * @retrun Ture if ADC is on else false
 */

static bool ADC_Init(ADC_config_t* config)
{
    assert(config != NULL);
    
#if defined ADC_INTERRUPT
    PIE1bits.ADIE = true;
    INTCONbits.PEIE = true;
    INTCONbits.GIE = true;
    PIR1bits.ADIF = false;
#elif defined ADC_POLLING
    PIE1bits.ADIE = false;
    INTCONbits.PEIE = false;
    PIR1bits.ADIF = false; 
#endif
    
    if(config->ADC_Enable)
    {
        ADCON1bits.ADFM = true;
#if  defined Analog_to_Digital_converter_12bit
        ADCON0bits.ADRMD = false;  //Resolutoin is 12 Bit
#elif defined Analog_to_Digital_converter_10bit
        ADCON0bits.ADRMD = true;  //Resolutoin is 10 Bit
#endif
        assert(config->conversion_clock <= (uint8_t)0x07 && config->conversion_clock >= (uint8_t)0x00);
        ADCON1bits.ADCS = config->conversion_clock; //ADC Conversion Clock Select bits
        assert(config->positive_voltage_reference <= (uint8_t)0x03 && config->positive_voltage_reference >= (uint8_t)0x00);
        ADCON1bits.ADPREF = config->positive_voltage_reference; //ADC Positive voltage Referecne Configuration bits
        ADCON1bits.ADNREF = config->negative_voltage_reference; //ADC negative voltage Referecne Configuration bits
        assert(config->positive_channel_select <= (uint8_t)0x1F && config->positive_channel_select >= (uint8_t)0x00);
        ADCON0bits.CHS = config->positive_channel_select;   //Positive Differental Input Channel Select bits
        assert(config->negative_channel_select <= (uint8_t)0x0F && config->negative_channel_select >= (uint8_t)0x00);
        ADCON2bits.CHSN = config->negative_channel_select;   //Negative Differental Input Channel Select bits
        ADCON0bits.ADON = config->ADC_Enable;   //To ebable ADC module must be set to a '1'
        __delay_ms(5);
        ADCON0bits.GO_nDONE = true;
        return true;
    }
    else
    {
        ADCON0bits.ADON = ADC_OFF;
        return false;
    }    
}

/* Fixed Voltage Reference module
 * @FVR abschalten
 * @FVR Selection bit
 */

static bool FVR_Init(FVR_config_t *config)
{
    assert(config != NULL);
    
    if(config->enable_fixed_voltage_reference)
    {
        FVRCONbits.FVREN = config->enable_fixed_voltage_reference;  //Fixed Voltage Reference Enable bit.
        assert(config->ADC_fixed_voltage_reference <= (uint8_t)0x03 && config->ADC_fixed_voltage_reference >= (uint8_t)0x00);
        FVRCONbits.ADFVR = config->ADC_fixed_voltage_reference; //ADC Fixed Voltage Reference Selection bit
        __delay_us(500);    //500 Micro Sekunden Zeit geben für zubereiten.
        return FVRCONbits.FVRRDY; //Fixed Voltage Reference output is ready for use.
    }
    return false;
}

/*
 * @return 16 Bit Value
 */

uint16_t ADC_RESULT(uint8_t _CHS)
{
#if defined ADC_INTERRUPT
    return ADC_result;
#elif defined ADC_POLLING
    if(CH_SEL != _CHS)   //if only the channel was changed.
    {
        ADCON0bits.CHS = _CHS;   //update the channel
        __delay_us(2); //When changing channels, a delay is required before starting the next conversation.
    }
    CH_SEL = _CHS;
    ADCON0bits.GO_nDONE = true;
    //LATBbits.LATB0 = true;
    while(ADCON0bits.GO_nDONE);
    //LATBbits.LATB0 = false;
#if defined Analog_to_Digital_converter_12bit
    ADC_result = (ADRESL + (ADRESH << 8) & 0xFFF);
#elif defined Analog_to_Digital_converter_10bit
    ADC_result = (ADRESL + (ADRESH << 8) & 0x3FF);
#endif
        ADCON0bits.GO_nDONE = true;
        PIR1bits.ADIF = false;
    return ADC_result;
#endif
    
}

#ifdef ADC_INTERRUPT

static void __interrupt() ADC(void)
{
    if((PIE1bits.ADIE & PIR1bits.ADIF)) //If the interrupt enabled and ADC flag set is
    {
#if defined Analog_to_Digital_converter_12bit
    ADC_result = (ADRESL + (ADRESH << 8) & 0xFFF);
#elif defined Analog_to_Digital_converter_10bit
    ADC_result = (ADRESL + (ADRESH << 8) & 0x3FF);
#endif
        ADCON0bits.GO_nDONE = true;
        PIR1bits.ADIF = false;
    }
}
#endif

#endif
//**************************************End ADC************************************************************

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
    68,    //PWM Frequenz 1.8kHz
};

//Look datasheet to configuration this the ADC
ADC_config_t adc_config = {
    ADC_ON, //ADC module abschalten
    0x00,   //Channel AN0 Ausgewählt  
    0x0F,   //Single ended ADC converter
    0x00,   //VREF+ is connected to VDD
    0x00,   //VREF- is connected to VSS
    0x02    //Fosc/32
};

//***********PWM Configuration****************
void PWM_Config(void)
{
    APFCONbits.CCP1SEL = false; //Select ouput pin for PWM (Pin C2)
    TRISC2 = true; //Disable the CCPx pin output driver by setting the associated TRIS bit.
    CCP1CONbits.CCP1M = (uint8_t)0xFF; //Configure the CCP module for PWM mode
    PWM_Duty(278); //Nicht relevant
    TIMER2_Init(&timer2_config); //Call the Timer 2 Config Function
    while(!PIR1bits.TMR2IF); //Wait until the Timer overflows
    TRISC2 = false; //Enable the CCPx pin output driver by clearing the associated TRIS bit.
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
    ANSELAbits.ANSA0 = true; TRISAbits.TRISA0 = true;
    
    //Pin B1 as Digital input
    ANSELBbits.ANSB1 = false; TRISBbits.TRISB1 = true;
    
    //Pins as Digital output
    ANSELBbits.ANSB2 = false; TRISBbits.TRISB2 = false;
    ANSELBbits.ANSB3 = false; TRISBbits.TRISB3 = false;
    
    //Pin zurucksetzen
    LATBbits.LATB2 = LATBbits.LATB3 = false; //Pins auf null setzen gemäss Aufgabe
}

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    ADC_Init(&adc_config);
    PWM_Config();
    
    //Endloss Schleife
    for(;;)
    {
        //Macro für Duty 8 Bit Auflösung aufrufen und ADC(A0) mit eine 8 Bit Auflösung ihn zuweisen
        PWM_Duty(ADC_RESULT(0));
        __delay_ms(1); //Verzögerung
    }
}

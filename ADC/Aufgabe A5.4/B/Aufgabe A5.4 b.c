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
#include "PBA_config.h"
//#include "LCD_2x16.h"

#define _XTAL_FREQ 8000000    // Frequenz 8MHz


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
/*
 * @parameter value 0..255
 * @non retrun
 */
void delay(uint8_t x)
{
    while(x--){
      __delay_ms(1);
    }
}

//Look datasheet to configuration this the ADC
ADC_config_t adc_config = {
    ADC_ON, //ADC module abschalten
    0x00,   //Channel AN0 Ausgewählt  
    0x0F,   //Single ended ADC converter
    0x00,   //VREF+ is connected to VDD
    0x00,   //VREF- is connected to VSS
    0x02    //Fosc/32
};

//Habe das FVR kurz getestet und hat funktioniert
/*FVR_config_t FVR_config = {
    1,
    0x01
};*/

void main(void) {
    
    //PORTs Initialisierung
    PORTs_Initialisierung();
    
    //Peripherie Initialisierung
    LCD_Init(V_3V3);
    //FVR_Init(&FVR_config);
    ADC_Init(&adc_config);
    
    static const uint8_t fix_vol_min = 39;  // 0.5V=(3.3V/255)*x -> x = (0.5V*255)/3.3V = 39
    static const uint8_t fix_vol_max = 116;  // 1.5V=(3.3V/255)*x -> x = (1.5V*255)/3.3V = 116
    static uint8_t hystrese = 39;
    
    for(;;)
    {
        
        uint8_t ADC0 = (uint8_t)(ADC_RESULT(0)>>2&(uint8_t)255);
        if((ADC0 >= fix_vol_min) && (ADC0 <= fix_vol_max))
        {
            LCD_Home();
            printf("Hysterese:%4imV",(uint16_t)(1000*(3.3/255.0)*ADC0));
            hystrese = ADC0;
        }
        
        uint8_t ADC1 = (uint8_t)(ADC_RESULT(1)>>2&(uint8_t)255);
        if(ADC1 >= fix_vol_max)
        LATD = 0xFF;
        else if(ADC1 <= (uint8_t)(fix_vol_max - hystrese))
        LATD = 0x00;
              
    }
}

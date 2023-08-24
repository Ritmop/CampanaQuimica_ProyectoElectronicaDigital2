#include "ADC_lib.h"

//Función para inicializar el módulo ADC
void adc_init(uint8_t J, uint8_t R, uint8_t clock, uint8_t channel){
    /*  
     bit J > A/D Conversion Result Format Select bit.
        1 = Right justified
        0 = Left justified
     bit R > Voltage Reference bit
        1 = Voltage from VREF- pin & VREF+ pin
        0 = Voltage from VSS & VDD
     */
    ADFM = J;   //Justification
    VCFG0 = R;  //Voltage reference
    VCFG1 = R;    
    //Conversion Clock Select bits
    switch(clock){
        case 1:
            ADCON0bits.ADCS = 0b00;
            //delay = 2.0us;
            break;
        case 4:
            ADCON0bits.ADCS = 0b01;
            //delay = 2.0us;
            break;
        case 8:
            ADCON0bits.ADCS = 0b10;
            //delay = 4.0us;
            break;
        case 20:
            ADCON0bits.ADCS = 0b11;
            //delay = 6.0us;
            break;
    }    
    ADCON0bits.CHS = channel;   //Select channel
    //PIE1bits.ADIE = 1;  //ADC Interrupt Enable
    //GIE  = 1;           //Global Interrupt Enable
    ADON = 1;           //Enable ADC Module    
}

//Función para leer ADC
uint16_t adc_read(void){    
    ADCON0bits.GO = 1;      //Start ADC conversion
    while(ADCON0bits.GO);   //Wait for conversion
    return (ADRESH<<8) | ADRESL;    //Return full result
}

//Función para cambiar de canal
void adc_sel_channel(uint8_t channel){
    ADCON0bits.CHS = channel;   //Select channel    
}

//Función para obtener el canal seleccionado
uint8_t adc_get_channel(void){
    return ADCON0bits.CHS;
}
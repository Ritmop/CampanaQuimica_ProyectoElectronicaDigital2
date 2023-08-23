/* 
 * File:   slave_sensors.c
 * Device: PIC16F887
 * Author: Judah Pérez - 21536
 *Compiler: XC8 (v2.41)
 * 
 * Program: Slave PIC for DHT11 sensor
 * Hardware: 
 * 
 * Created: Aug 18, 2023
 * Last updated:
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include "I2C.h"
#include "ADC_lib.h"
//#include "LCD4b.h"

/*---------------------------- CONFIGURATION BITS ----------------------------*/
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

/*----------------------- GLOBAL VARIABLES & CONSTANTS -----------------------*/
#define _XTAL_FREQ      8000000
#define address_MQ2_IR  0x20
#define MQ2chan     0   //AN0
#define LM35chan    1   //AN1

uint8_t discard;
uint8_t send_data;
uint8_t request;

uint8_t MQ2_val;    //MQ2 read value
uint8_t LM35_val;    //LM35 read value
uint8_t IR_sens;    //IR sensor
//char MQ2_s[] = {'0','0','0','\0'};
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
uint8_t map(uint8_t val, uint8_t min1, uint8_t max1, uint8_t min2, uint16_t max2);
//void LDC_output(void);
//void separar_digitos8(uint8_t num, char dig8[]);
/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    if(SSPIF){ 

        CKP = 0; //Hold clock in low to ensure data setup time
       
        if (SSPOV || WCOL ){ //Received overflow or Write collision
            discard = SSPBUF;// Discard value by reading the buffer
            SSPOV = 0;       // Clear the overflow flag
            WCOL = 0;        // Clear the collision bit
            CKP = 1;         // Enables SCL (Clock)
        }
        
        if(!D_nA && !R_nW) {    //Received an Address and Write
            //__delay_us(7);
            discard = SSPBUF;   // Discard address by reading the buffer
            //__delay_us(2);
            SSPIF = 0;
            CKP = 1;
            while(!BF);         // Wait to receive data
            request = SSPBUF;   // Store data
            __delay_us(250);
        }
        else if(!D_nA && R_nW){//Received an Address and Read
            discard = SSPBUF;   //Discard address by reading the buffer
            BF = 0;
            SSPBUF = send_data;     //Load data to buffer
            CKP = 1;
            __delay_us(250);
            while(BF);          //Wait until buffer is cleared
        }
        
        SSPIF = 0;    
    }
}

/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    while(1){
        //Loop
        //Read sensors value
        if(adc_get_channel() == MQ2chan){
            MQ2_val = (adc_read()>>8) & 0x00FF; //Read MQ2 analog value
            adc_sel_channel(LM35chan);  //Switch channel
        }
        else if (adc_get_channel() == LM35chan){
            LM35_val = (adc_read()>>8) & 0x00FF; //Read LM35 analog value 
            adc_sel_channel(MQ2chan);   //Switch channel
        }
        PORTB = LM35_val;
                
        IR_sens = RA2;   //Read IR digital value
        
        switch(request){
            case 'G':   //Gas
                send_data = MQ2_val;
                break;
            case 'T':   //Temperature
                send_data = LM35_val;
                break;
            case 'I':   //Infrared
                send_data = IR_sens;
                break;            
            default:
                send_data = 0xFF;
                break;
        }
        
        //Display data
        //LDC_output();
        __delay_ms(10);
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    ANSEL = 3;  //RA0 & RA1 as analog (AN0)
    ANSELH= 0;
    TRISA = 255;  //PORTA as input
    
    TRISB = 0;
    PORTB = 0;
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize ADC
    adc_init(0, 0, 8, 0); //Initialize ADC. Left, Vdd/Vss, 8MHz, AN0.
    
    //Initialize I2C Com    
    I2C_Slave_Init(address_MQ2_IR);
    
    //Initialize LCB 4bit mode
//    Lcd_Init();
//    __delay_ms(10);
}

//void LDC_output(void){
//    separar_digitos8(MQ2_val, MQ2_s);
//    
//    Lcd_Set_Cursor(1,1);
//    Lcd_Write_String("G: ");
//    Lcd_Write_String(MQ2_s);
//    Lcd_Set_Cursor(2,1);
//    if(IR_sens)
//        Lcd_Write_String("PUERTA ABIERTA");
//    else
//        Lcd_Write_String("PUERTA CERRADA");
//    
//}

//void separar_digitos8(uint8_t num, char dig8[]){
//    uint8_t div1,div2,div3,centenas,decenas,unidades;
//    div1 = num / 10;
//    unidades = num % 10;
//    div2 = div1 / 10;
//    decenas = div1 % 10;  
//    centenas = div2 % 10;
//    
//    dig8[2] = unidades + '0';
//    dig8[1] = decenas  + '0';
//    dig8[0] = centenas + '0';
//}

uint8_t map(uint8_t val, uint8_t min1, uint8_t max1, uint8_t min2, uint16_t max2){
    return ((val-min1)*(max2-min2)/(max1-min1))+min2;
}
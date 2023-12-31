/* 
 * File:   slave_sensors.c
 * Device: PIC16F887
 * Author: Judah Sebastian P�rez Zeiset - 21536
 *         Carlos Daniel Valdez Coreas - 21976
 *Compiler: XC8 (v2.41)
 * 
 * Program: Slave PIC for DHT11 sensor
 * Hardware:
 *          SCL and SDA connected to Master
 *          MQ2 sensor on RA0
 *          LM35 on RA1
 * 
 * Created: Aug 18, 2023
 * Last updated: Aug 24, 2023
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include "I2C.h"
#include "ADC_lib.h"

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

#define address_sensors  0x20    //I2C address
#define MQ2chan     0   //AN0
#define LM35chan    1   //AN1

uint8_t discard;    //I2C read and write data
uint8_t send_data;
uint8_t request;

uint8_t MQ2_val;    //MQ2 read value
uint8_t LM35_val;   //LM35 read value

/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);

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
        
        switch(request){
            case 'G':   //Gas
                send_data = MQ2_val;
                break;
            case 'T':   //Temperature
                send_data = LM35_val;
                break;            
            default:
                send_data = 0xFF;
                break;
        }
        
        __delay_ms(10);
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    ANSEL = 3;  //RA0 & RA1 as analog (AN0)
    ANSELH= 0;
    TRISA = 255;  //PORTA as input
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize ADC
    adc_init(0, 0, 8, 0); //Initialize ADC. Left, Vdd/Vss, 8MHz, AN0.
    
    //Initialize I2C communication
    I2C_Slave_Init(address_sensors);
}
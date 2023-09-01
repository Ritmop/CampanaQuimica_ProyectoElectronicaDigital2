/* 
 * File:   slave_DHT11.c
 * Device: PIC16F887
 * Author: Judah Sebastian Pérez Zeiset - 21536
 *         Carlos Daniel Valdez Coreas - 21976
 *Compiler: XC8 (v2.41)
 * 
 * Program: Slave PIC for DHT11 sensor
 * Hardware: 
 *          SCL and SDA connected to Master
 *          DHT11 sensor on RD0 
 * 
 * Created: Aug 18, 2023
 * Last updated: Aug 24, 2023
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include "I2C.h"
#include "DHT11.h"

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

#define address_DHT11   0x10    //I2C address

uint8_t discard;    //I2C read and write data
uint8_t send_data;
uint8_t request;

int8_t data_ok;     //DHT11 data received successfully
int16_t humedad, temperatura;   //DHT11 sensor values

uint8_t counter = 250;  //Slow-rate sensor request

/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);

/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    if(SSPIF){ 

        CKP = 0; //Hold clock in low to ensure data setup time
       
        if (SSPOV || WCOL ){    //Received overflow or Write collision
            discard = SSPBUF;   //Discard value by reading the buffer
            SSPOV = 0;          //Clear the overflow flag
            WCOL = 0;           //Clear the collision bit
            CKP = 1;            //Enables SCL (Clock)
        }
        
        if(!D_nA && !R_nW) {    //Received an Address and Write
            //__delay_us(7);
            discard = SSPBUF;   //Discard address by reading the buffer
            //__delay_us(2);
            SSPIF = 0;
            CKP = 1;
            while(!BF);         //Wait to receive data
            request = SSPBUF;   //Store data
            __delay_us(250);
        }
        else if(!D_nA && R_nW){ //Received an Address and Read
            discard = SSPBUF;   //Discard address by reading the buffer
            BF = 0;
            SSPBUF = send_data; //Load data to buffer
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
    DHT11_start();  //Prepare DHT11
    __delay_ms(100);
    while(1){
        //Loop
        
        //Request data to sensor every 2500ms (2.5s)
        if(counter >= 250){
            data_ok = DHT11_read_data(&humedad, &temperatura);
            counter = 0;
        }        
        
        //Prepare requested data
        switch(request){
            case 'T':   //Entero temperatura
                send_data = (temperatura & 0xFF00)>>8;
                break;
            case 't':   //Decimal temperatura
                send_data = (temperatura & 0x00FF);
                break;
            case 'H':   //Entero humedad
                send_data = (humedad & 0xFF00)>>8;
                break;
            case 'h':   //Decimal humedad
                send_data = (humedad & 0x00FF);
                break;
            default:
                send_data = 0xFF;
                break;
        }

        counter++;
        __delay_ms(10);       
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH= 0;
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize I2C communication
    I2C_Slave_Init(address_DHT11);
}
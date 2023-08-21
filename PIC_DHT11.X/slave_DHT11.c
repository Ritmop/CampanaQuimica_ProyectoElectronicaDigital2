/* 
 * File:   slave_DHT11.c
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
#include "DHT11.h"
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
#define address_DHT11   0x10

uint8_t discard;
uint8_t send_data;
uint8_t request;

int8_t data_ok = 1;
int16_t humedad, temperatura;
//char u_temp[3];
//char d_temp[3];
//char u_hum[3];
//char d_hum[3];
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void LDC_output(void);
void separar_digitos8(uint8_t num, char dig8[]);
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
    DHT11_start();  //Prepare DHT11
    while(1){
        //Loop
        __delay_ms(100);
        //Request data to sensor
        data_ok = DHT11_read_data(&humedad, &temperatura);
        
        //Prepare requested data
        PORTB = request;
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
                send_data = 'X';
                break;
        }
        PORTA = send_data;
        
//        //Data request successful 
//        if(data_ok){
//            Lcd_Clear();
//            LDC_output();
//        }
//       //Data request not successful
//        else {
//            Lcd_Set_Cursor(2,1);
//            Lcd_Clear();
//            Lcd_Write_String("READ ERROR");
//        }           
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH= 0;
    
    TRISA = 0;
    PORTA = 0;
    TRISB = 0;
    PORTB = 0;
    
    TRISD = 0;  //LCD output
    PORTD = 0;
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize I2C Com    
    I2C_Slave_Init(address_DHT11);
    
//    //Initialize LCB 4bit mode
//    Lcd_Init();
//    __delay_ms(10);
}
//
//void LDC_output(void){
//    //Separar enteros y decimales
//    separar_digitos8((temperatura & 0xFF00)>>8,u_temp);
//    separar_digitos8((temperatura & 0x00FF),d_temp);
//    separar_digitos8((humedad & 0xFF00)>>8,u_hum);
//    separar_digitos8((humedad & 0x00FF),d_hum);
//    
//    Lcd_Set_Cursor(1,1);
//    Lcd_Write_String("T: ");
//    Lcd_Write_String(u_temp);
//    Lcd_Write_Char('.');
//    Lcd_Write_String(d_temp);
//    Lcd_Write_String("'C");
//    
//    Lcd_Set_Cursor(2,1);
//    Lcd_Write_String("H:  ");
//    Lcd_Write_String(u_hum);
//    Lcd_Write_String(" %RH");
//    
//}
//
//void separar_digitos8(uint8_t num, char dig8[]){
//    uint8_t div1,decenas,unidades;
//    div1 = num / 10;
//    unidades = num % 10;
//    decenas = div1 % 10;    
//    
//    dig8[1] = unidades + 0x30;
//    dig8[0] = decenas  + 0x30;
//}
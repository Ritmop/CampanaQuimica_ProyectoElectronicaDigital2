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
//#include "I2C.h"
#include "DHT11.h"
#include "LCD4b.h"

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
#define _XTAL_FREQ 8000000

int8_t data_ok = 1;
int16_t humedad, temperatura;
char u_temp[3];
char d_temp[3];
char u_hum[3];
char d_hum[3];
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void LDC_output(void);
void separar_digitos8(uint8_t num, char dig8[]);
/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    
}

/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    DHT11_start();  //Prepare DHT11
    while(1){
        //Loop
        __delay_ms(2000);
        //Request data to sensor
        data_ok = DHT11_read_data(&humedad, &temperatura);
        
        //Data request successful 
        if(data_ok){
            Lcd_Clear();
            LDC_output();
        }
       //Data request not successful
        else {
            Lcd_Set_Cursor(2,1);
            Lcd_Clear();
            Lcd_Write_String("READ ERROR");
        }           
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH= 0;
    
    TRISD = 0;  //LCD output
    PORTD = 0;
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize LCB 4bit mode
    Lcd_Init();
    __delay_ms(10);
}

void LDC_output(void){
    //Separar enteros y decimales
    separar_digitos8((temperatura & 0xFF00)>>8,u_temp);
    separar_digitos8((temperatura & 0x00FF),d_temp);
    separar_digitos8((humedad & 0xFF00)>>8,u_hum);
    separar_digitos8((humedad & 0x00FF),d_hum);
    
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("T: ");
    Lcd_Write_String(u_temp);
    Lcd_Write_Char('.');
    Lcd_Write_String(d_temp);
    Lcd_Write_String("'C");
    
    Lcd_Set_Cursor(2,1);
    Lcd_Write_String("H:  ");
    Lcd_Write_String(u_hum);
    Lcd_Write_String(" %RH");
    
}

void separar_digitos8(uint8_t num, char dig8[]){
    uint8_t div1,decenas,unidades;
    div1 = num / 10;
    unidades = num % 10;
    decenas = div1 % 10;    
    
    dig8[1] = unidades + 0x30;
    dig8[0] = decenas  + 0x30;
}
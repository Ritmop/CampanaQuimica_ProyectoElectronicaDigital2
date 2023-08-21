/*
 * File:   master.c
 * Device: PIC16F887
 * Author: Judah Pérez - 21536
 *Compiler: XC8 (v2.41)
 * 
 * Program: Master PIC
 * Hardware: 
 * 
 * Created: Aug 20, 2023
 * Last updated:
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include "I2C.h"
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
#define _XTAL_FREQ      8000000
#define address_DHT11   0x10
#define address_MQ2_IR  0x20
#define address_motor   0x30
#define read  1
#define write 0

uint8_t u_temp,d_temp,u_hum,d_hum,gas,ired;    //Sensors data
char Su_temp[3];    //Sensor data as strings
char Sd_temp[3];
char Su_hum[3];
char Sd_hum[3];
char Sgas[4];
char Sired[2];
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void requestTemp(void);
void requestHum(void);
void requestGas(void);
void requestIR(void);
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
    while(1){
        //Loop
        requestTemp();
        requestHum();
        requestGas();
        requestIR();
        LDC_output();
        __delay_ms(2500);
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH= 0;
    
    TRISA = 0;
    PORTA = 0;
    
    TRISD = 0;  //LCD output
    PORTD = 0;
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize LCB 4bit mode
    Lcd_Init();
    __delay_ms(10);
    
    // Initialize I2C Com
    I2C_Master_Init(100000);        
}

void requestTemp(void){
    //Entero temperatura
    I2C_Master_Start();
    I2C_Master_Write(address_DHT11+write);
    I2C_Master_Write('T');
    __delay_ms(200);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_DHT11+read);
    u_temp = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(500);
    
    //Decimal temperatura
    I2C_Master_Start();
    I2C_Master_Write(address_DHT11+write);
    I2C_Master_Write('t');
    __delay_ms(200);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_DHT11+read);
    d_temp = I2C_Master_Read(0);
    I2C_Master_Stop(); 
    __delay_ms(500);
}

void requestHum(void){
    //Entero humedad
    I2C_Master_Start();
    I2C_Master_Write(address_DHT11+write);
    I2C_Master_Write('H');
    __delay_ms(200);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_DHT11+read);
    u_hum = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(500);
}

void requestGas(void){
    I2C_Master_Start();
    I2C_Master_Write(address_MQ2_IR+write);
    I2C_Master_Write('G');
    __delay_ms(200);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_MQ2_IR+read);
    gas = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(500);
}

void requestIR(void){
    I2C_Master_Start();
    I2C_Master_Write(address_MQ2_IR+write);
    I2C_Master_Write('I');
    __delay_ms(200);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_MQ2_IR+read);
    ired = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(500);
}

void LDC_output(void){
    //Separar enteros y decimales
    separar_digitos8(u_temp,Su_temp);
    separar_digitos8(d_temp,Sd_temp);
    separar_digitos8(u_hum,Su_hum);
    //separar_digitos8(d_hum,Sd_hum);
    separar_digitos8(gas,Sgas);
    separar_digitos8(ired,Sired);
    
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("T:");
    Lcd_Write_String(Su_temp);
    Lcd_Write_Char('.');
    Lcd_Write_String(Sd_temp);
    Lcd_Write_String("'C");
    
    Lcd_Set_Cursor(2,1);
    Lcd_Write_String("H:");
    Lcd_Write_String(Su_hum);
    Lcd_Write_String("%RH");
    
    Lcd_Set_Cursor(1,12);
    Lcd_Write_String("G:");
    Lcd_Write_String(Sgas);
    
    Lcd_Set_Cursor(2,12);
    Lcd_Write_String("IR:");
    Lcd_Write_String(Sired);
}

void separar_digitos8(uint8_t num, char dig8[]){
    uint8_t div1,div2,centenas,decenas,unidades;
    div1 = num / 10;
    unidades = num % 10;
    decenas = div1 % 10;
    div2 = div1 / 10;
    centenas = div2 % 10;
    
    if(decenas == 0){
        dig8[0] = unidades  + '0';
    }
    else if(centenas == 0){
        dig8[1] = unidades + '0';
        dig8[0] = decenas  + '0';
    }
    else{
        dig8[2] = unidades + '0';
        dig8[1] = decenas  + '0';
        dig8[0] = centenas + '0';
    }
}
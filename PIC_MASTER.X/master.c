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
#define address_motors  0x30
#define read  1
#define write 0
#define thresTemp       80  //Temperature threshold (°C)
#define thresGas        100 //Gas threshold

uint8_t u_temp,d_temp,u_hum,d_hum,gas,ired;    //Sensors data
uint8_t tempC, gasPPM;
char Su_temp[4];    //Sensor data as strings
char Sd_temp[3];
char Su_hum[3];
char Sd_hum[3];
char Sgas[4];
char Sired[2];

uint8_t counter;    //Contador para lectura de sensores lentos
uint8_t servoPos = 180;   //Servo position
uint8_t motorCon;   //Motor control (High nibble: motor DC, Low nibble: Servo)
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void requestTemp(void);
void requestHum(void);
void requestGas(void);
void requestIR(void);
void writeMotors(void);
void LDC_output(void);
void num_to_string(uint16_t num, char dig8[], uint8_t len);
uint16_t map(uint8_t val, uint8_t min1, uint8_t max1, uint8_t min2, float max2);
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
        if (counter >= 10){ //Request every 2500 ms (2.5 s)
            requestHum();           
            counter = 0;
        }
        requestTemp();
        requestGas();
        requestIR();
        
        writeMotors();
        
        LDC_output();
        
        __delay_ms(250);
        counter++;
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
    I2C_Master_Start();
    I2C_Master_Write(address_MQ2_IR+write);
    I2C_Master_Write('T');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_MQ2_IR+read);
    u_temp = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);    
    //Convertir valor analogico a grados centígrados
    tempC = map(u_temp,0,77,0,150);
}

void requestHum(void){
    I2C_Master_Start();
    I2C_Master_Write(address_DHT11+write);
    I2C_Master_Write('H');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_DHT11+read);
    u_hum = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);
}

void requestGas(void){
    I2C_Master_Start();
    I2C_Master_Write(address_MQ2_IR+write);
    I2C_Master_Write('G');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_MQ2_IR+read);
    gas = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);    
    //Convertir valor analogico a ppm
    gasPPM = map(gas,0,255,0,800);
}

void requestIR(void){
    I2C_Master_Start();
    I2C_Master_Write(address_MQ2_IR+write);
    I2C_Master_Write('I');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_MQ2_IR+read);
    ired = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);
}

void writeMotors(void){
    //Check conditions for water pump
    if(u_temp > thresTemp && gas > thresGas)
        motorCon |= 0x10;   //Set DC, copy Servo
    
    else
        motorCon &= 0x0F;   //Reset DC, copy Servo
    
    //Update servo position
    //Position = 2 ? 0deg, 3 ? 90deg, 4 ? 180deg
    switch(servoPos){
        case 0:
            motorCon &= 0xF2;   //Copy DC, set servo
            break;
        case 90:
            motorCon &= 0xF3;   //Copy DC, set servo
            break;
        case 180:
            motorCon &= 0xF4;   //Copy DC, set servo
            break;
        default:
            motorCon &= 0xF2;   //Default position 0 deg
            break;
    }
    
    //Send motor control register to slave
    I2C_Master_Start();
    I2C_Master_Write(address_motors+write);
    I2C_Master_Write(motorCon);
    I2C_Master_Stop();
    __delay_ms(20);
}

void LDC_output(void){
    Lcd_Clear();
    //Integer to string conversion
    num_to_string(tempC,Su_temp,3);
    num_to_string(u_hum,Su_hum,2);
    num_to_string(gasPPM,Sgas,3);
    num_to_string(ired,Sired,1);
    
    //Display in LCD
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("T:");
    Lcd_Write_String(Su_temp);
    //Lcd_Write_Char('.');
    //Lcd_Write_String(Sd_temp);
    Lcd_Write_String("'C");
    
    Lcd_Set_Cursor(2,1);
    Lcd_Write_String("H:");
    Lcd_Write_String(Su_hum);
    Lcd_Write_String("%RH");
    
    Lcd_Set_Cursor(1,9);
    Lcd_Write_String("G:");
    Lcd_Write_String(Sgas);    
    Lcd_Write_String("ppm");
    
    Lcd_Set_Cursor(2,9);
    Lcd_Write_String("IR:");
    Lcd_Write_String(Sired);
}

void num_to_string(uint16_t num, char dig8[], uint8_t len){
    uint16_t div1,div2,div3,miles,centenas,decenas,unidades;
    div1 = num / 10;
    unidades = num % 10;    
    div2 = div1 / 10;
    decenas = div1 % 10;
    div3 = div2 / 10;
    centenas = div2 % 10;
    miles = div3 % 10;
    
    if(len == 1){
        dig8[0] = unidades + '0';
    }
    else if(len == 2){
        dig8[1] = unidades + '0';
        dig8[0] = decenas  + '0';
    }
    else if (len == 3){
        dig8[2] = unidades + '0';
        dig8[1] = decenas  + '0';
        dig8[0] = centenas + '0';
    }
    else if (len == 4){
        dig8[3] = unidades + '0';
        dig8[2] = decenas  + '0';
        dig8[1] = centenas + '0';
        dig8[0] = miles    + '0';
    }
}

uint16_t map(uint8_t val, uint8_t min1, uint8_t max1, uint8_t min2, float max2){
    return ((val-min1)*(max2-min2)/(max1-min1))+min2;
}
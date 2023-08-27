/*
 * File:   master.c
 * Device: PIC16F887
 * Author: Judah Sebastian Pérez Zeiset - 21536
 *         Carlos Daniel Valdez Coreas - 21976
 *Compiler: XC8 (v2.41)
 * 
 * Program: Master PIC
 * Hardware:
 *          4.7kOhm pull-up resistors on SCL and SDA pins
 *          LCD on PORTD[2:7]
 *          SCL and SDA connected to 3 slave PIC16F887
 *          TX and RX connected to ESP32
 * 
 * Created: Aug 20, 2023
 * Last updated: Aug 24, 2023
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include "I2C.h"
#include "LCD4b.h"
#include "UART.h"

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

#define address_DHT11   0x10    //Slave address for I2C
#define address_sensors 0x20
#define address_motors  0x30

#define read  1     //Add to I2C address to read/write
#define write 0

#define thresTemp       50  //Temperature threshold (°C)
#define thresGas        400 //Gas threshold (PPM)

uint8_t n_temp,n_hum,n_gas;  //Sensors data as numbers

uint8_t  tempC;     //Sensor data mapped values
uint16_t gasPPM;

char S_temp[4];    //Sensor data as strings
char S_hum [3];
char S_gas [4];

uint8_t counter;    //Slow-rate sensor request
uint8_t servoPos;   //Servo position
uint8_t motorCon;   //Motor control (High nibble: DC motor, Low nibble: Servo)

/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);

void requestTemp(void); //Request sensor data through I2C
void requestHum(void);
void requestGas(void);

void writeMotors(void); //Send data to motor through I2C

void LDC_output(void);  //Display sensor data on LDC

void sendDataUART(void);//Send sensor data through UART

void num_to_string(uint16_t num, char dig8[], uint8_t len);//Number to string conversion
uint16_t map(uint8_t val, uint8_t min1, uint8_t max1, uint8_t min2, long max2);

/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    if(RCIF){        
        servoPos  = UART_read_char(); //Receive Servo position from UART
        RCIF = 0;
    }
}
/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    while(1){
        //Loop
        //Data request
        if (counter >= 25){ //Request after more than 2500ms
            requestHum();           
            counter = 0;
        }
        requestTemp();
        requestGas();
        
        //Data write
        writeMotors();
        
        LDC_output();   //Display sensors data on LDC
        sendDataUART(); //Send data to ESP32
        
        counter++;
        __delay_ms(100);
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
    
    // Initialize I2C communication
    I2C_Master_Init(100000);
    
    //Initialize UART    
    UART_RX_config(9600);
    UART_TX_config(9600);
}

void requestTemp(void){
    //Request Temperature to PIC_SENSORS
    I2C_Master_Start();
    I2C_Master_Write(address_sensors+write);
    I2C_Master_Write('T');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_sensors+read);
    n_temp = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);    
    //Map analog value to Celsius
    tempC = map(n_temp,0,77,0,150);
}

void requestHum(void){
    //Request Humidity to PIC_SENSORS
    I2C_Master_Start();
    I2C_Master_Write(address_DHT11+write);
    I2C_Master_Write('H');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_DHT11+read);
    n_hum = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);
}

void requestGas(void){
    //Request Gas particles to PIC_SENSORS
    I2C_Master_Start();
    I2C_Master_Write(address_sensors+write);
    I2C_Master_Write('G');
    __delay_ms(20);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(address_sensors+read);
    n_gas = I2C_Master_Read(0);
    I2C_Master_Stop();
    __delay_ms(20);    
    //Map analog value to PPM
    gasPPM = map(n_gas,0,255,100,800);
}

void writeMotors(void){
    //Check conditions for water pump
    if(tempC > thresTemp && gasPPM > thresGas)
        motorCon |= 0x10;   //Set DC, copy Servo
    else
        motorCon &= 0x0F;   //Reset DC, copy Servo
    
    //Update motor control register to match new servo position
    //Position = 0 -> 0deg, 1 -> 90deg, 2 -> 180deg
    motorCon &= 0xF0;   //Copy DC, clear servo
    switch(servoPos){
        case '0':
            motorCon |= 0x02;   //Copy DC, set servo
            break;
        case '1':
            motorCon |= 0x03;   //Copy DC, set servo
            break;
        case '2':
            motorCon |= 0x04;   //Copy DC, set servo
            break;
        default:
            motorCon |= 0x02;   //Default position 0 deg
            break;
    }
    
    //Send motor control register to PIC_MOTORS
    I2C_Master_Start();
    I2C_Master_Write(address_motors+write);
    I2C_Master_Write(motorCon);
    I2C_Master_Stop();
    __delay_ms(20);
}

void LDC_output(void){
    Lcd_Clear();
    //Number to string conversion
    num_to_string(tempC,S_temp,3);
    num_to_string(n_hum,S_hum,2);
    num_to_string(gasPPM,S_gas,3);
    
    //Display on LCD
    Lcd_Set_Cursor(1,1);    //Temperature (°C)
    Lcd_Write_String("T:");
    Lcd_Write_String(S_temp);
    Lcd_Write_String("^C");
    
    Lcd_Set_Cursor(2,1);    //Humidity (%RH)
    Lcd_Write_String("H:");
    Lcd_Write_String(S_hum);
    Lcd_Write_String("%RH");
    
    Lcd_Set_Cursor(1,9);    //Gas (ppm)
    Lcd_Write_String("G:");
    Lcd_Write_String(S_gas);    
    Lcd_Write_String("ppm");
    
    Lcd_Set_Cursor(2,9);    //Infrared (Digital)
    Lcd_Write_String("PUERTA:");
}

void sendDataUART(void){
    //Data sent to ESP32
    UART_write_char('\n');
    UART_write_char(tempC);
    UART_write_char(' ');
    UART_write_char(n_hum);
    UART_write_char(' ');
    UART_write_char((gasPPM & 0xFF00) >> 8);
    UART_write_char(gasPPM & 0x00FF);
    UART_write_char(' ');
    __delay_ms(500);
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
}

uint16_t map(uint8_t val, uint8_t min1, uint8_t max1, uint8_t min2, long max2){
    return ((val-min1)*(max2-min2)/(max1-min1))+min2;
}   
/* 
 * File:   slave_motors.c
 * Device: PIC16F887
 * Author: Judah Sebastian Pérez Zeiset - 21536
 *         Carlos Daniel Valdez Coreas - 21976
 *Compiler: XC8 (v2.40)
 * 
 * Program: Slave PIC for Motor Control
 * Hardware:
 *          SCL and SDA connected to Master
 *          DC motor on RC0
 *          Servomotor on RC2
 * 
 * Created: Aug 21, 2023
 * Last updated: Aug 24, 2023
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include "I2C.h"

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
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

/*----------------------- GLOBAL VARIABLES & CONSTANTS -----------------------*/
#define _XTAL_FREQ      8000000
#define address_motors  0x30
#define TMR0_n          131 //TMR0 Interrupt set to 50Hz
#define servoPin        RC2
#define motorPin        RC0

uint8_t discard;
uint8_t send_data;
uint8_t readI2C;
uint8_t TMR0count = 0;

uint8_t servoPos = 2;
        
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void initPWM(void);
void angle_to_PWM(uint8_t angle);

/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr (void){
    if(SSPIF){ 
        CKP = 0; //Hold clock in low to ensure data setup time
       
        if (SSPOV || WCOL ){ //Received overflow or Write collision
            discard = SSPBUF;// Discard value by reading the buffer
            SSPOV = 0;       // Clear the overflow flag
            WCOL = 0;        // Clear the collision bit
            CKP = 1;         // Enables SCL (Clock)
        }
        
        if(!D_nA && !R_nW) {    //Received an Address and Write
            __delay_us(7);
            discard = SSPBUF;   // Discard address by reading the buffer
            __delay_us(2);
            SSPIF = 0;
            CKP = 1;
            while(!BF);         // Wait to receive data
            readI2C = SSPBUF;   // Store data
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
    
    if(T0IF){
        TMR0count++;
        TMR0 = TMR0_n;
        T0IF = 0;
    }
}

/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- SETUP ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    while(1){
        //Loop
        
        //Servo position
        servoPos = readI2C & 0x0F;
        
        angle_to_PWM(servoPos); 
        
        //DC motor state
        if((readI2C & 0xF0) == 0x10)
            motorPin = 1;  //Activate motor
        else
            motorPin = 0;  //Deactivate motor
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
   //I/O CONFIG    
    ANSEL = 0;
    ANSELH= 0;
    
    TRISC0 = 0; //DC Motor
    RC0 = 0;
    TRISC2 = 0; //Servomotor
    RC2 = 0;
    
    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //Initialize I2C Com    
    I2C_Slave_Init(address_motors);
    
    //Initialize PWM with TMR0
    initPWM();
}

void initPWM(void){
    //TMR0 init
    T0CS = 0;   //Internal instruction cycle clock (FOSC/4)
    PSA = 0;    //Prescaler to TMR0
    OPTION_REGbits.PS = 0b010;     //Prescaler 1:8
    T0IE = 1;   //TMR0 Interrupt enable
    T0IF = 0;   //Reset flag
    GIE = 1;    //Global Interrupt Enable
    TMR0 = TMR0_n;
}

void angle_to_PWM(uint8_t position){
    //Position = 2 -> 0deg, 3 -> 90deg, 4 -> 180deg
    if(TMR0count >= 40){
        TMR0count = 0;                        
        servoPin = 1;
    }
    else if (TMR0count == position){                        
        servoPin = 0;           
    }
}
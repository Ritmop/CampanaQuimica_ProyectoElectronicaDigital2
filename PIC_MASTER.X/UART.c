#include "UART.h"

char prev;

//Función para inicializar recepcion UART
void UART_RX_config (long baudrate){
    //12.1.2.8 Asynchronous Reception Set-up
    //Converir baudrate
    long BR;
    BR = ((_XTAL_FREQ/baudrate)/64) - 1;
    SPBRG = BR & 0x00FF;
    SPBRGH = (BR & 0xFF00) >> 8;
    //Puerto serial asincrono
    SYNC = 0;
    SPEN = 1;
    //Habilitar recepcion por interrupciones
    CREN = 1;
    RCIE = 1;
    PEIE = 1;
    GIE = 1;
    RCIF = 0;
}

//Función para inicializar transmision UART
void UART_TX_config (long baudrate){
    //12.1.1.6 Asynchronous Transmission Set-up
    //Converir baudrate
    long BR;
    BR = ((_XTAL_FREQ/baudrate)/64) - 1;
    SPBRG = BR & 0x00FF;
    SPBRGH = (BR & 0xFF00) >> 8;
    //Puerto serial asincrono
    SYNC = 0;
    SPEN = 1;
    //Habilitar transmision
    TXEN = 1;
    TXIE = 0;
    TXIF = 0;   //Clear flag
}

//Funcion para transmitir un caracter
void UART_write_char(char c){
    while(!TXIF);
    TXIF = 0;
    TXREG = c;
}

//Funcion para transmitir una cadena
void UART_write_string(char *s){
    int i;
    for (i = 0; s[i] != '\0'; i++)
        UART_write_char(s[i]); 
}

//Funcion para leer un caracter a la vez
//Interrupciones necesarias
char UART_read_char(){
//    PORTA = prev;
//    if(RCREG == prev){
//        RE2 = 1;
//        return 0x00;        
//    }
//    else{
//        RE2 = 0;
//        prev = RCREG;
//        return RCREG;
//    }
    return RCREG;
}
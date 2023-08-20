/* #include <DHT11.c>
*
* Creada por: Ing. Abiezer Hernandez O.
* Fecha de creacion: 10/04/2019
* Electronica y Circuitos
* 
* Modificaciones: Judah Pérez
*
*/

#include "DHT11.h"

int8_t bits[5];

void DHT11_start(){
    TRISD0 = 0;
    DHT11_PIN = 0;
    __delay_ms(25);
    DHT11_PIN = 1;
    TRISD0 = 1;
    __delay_us(30);
}

void DHT11_response(){
    while(DHT11_PIN == 1);
    while(DHT11_PIN == 0);
    while(DHT11_PIN == 1);   
}

int8_t DHT11_read_byte()
{
   int8_t i;
   int8_t data = 0;  
   for(i=0; i<8; i++){
    while(DHT11_PIN == 0);
    __delay_us(30);         
    if(DHT11_PIN == 1){
        data = (data<<1 | 1); 
    }
    else{
        data = (data<<1);  
    }
    while(DHT11_PIN == 1);
   }
   return data;
}

int8_t DHT11_read_data(int16_t *hum, int16_t *temp)
{
   int8_t parity = 0;
   DHT11_start();
   DHT11_response();   
   bits[0] = DHT11_read_byte();   //Humedad entero
   bits[1] = DHT11_read_byte();   //Humedad decimal
   bits[2] = DHT11_read_byte();   //Temp entero
   bits[3] = DHT11_read_byte();   //Temp decimal
   bits[4] = DHT11_read_byte();   //Paridad
   parity = bits[0] + bits[1] + bits[2] + bits[3];
   if (parity == bits[4]){       
        *hum = bits[0]<<8 | bits[1];
        *temp = bits[2]<<8 | bits[3];  
    return 1;
   }
   else{
      return 0;
   }
}

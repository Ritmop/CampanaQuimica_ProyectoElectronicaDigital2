/* 
 * File:   DHT11.h
 * Author: Ing. Abiezer Hernandez O.
 * Modificada: Judah Pérez
 *
 * Created on August 18, 2023, 2:08 PM
 */

#ifndef DHT11_H
#define	DHT11_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif

#ifndef DHT11_PIN
#define DHT11_PIN RD0
#endif

#include <xc.h>

//DHT11 functions developed by Ing. Abiezer Hernandez O.

void DHT11_start(void);

void DHT11_response(void);

int8_t DHT11_read_byte(void);

int8_t DHT11_read_data(int16_t *hum, int16_t *temp);

#endif	/* DHT11_H */


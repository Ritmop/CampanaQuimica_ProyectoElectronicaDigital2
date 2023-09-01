// Proyecto 1 Electrónica Digital II
// Judah Pérez (21536) y Carlos Daniel Valdez (21976)
// Programa para leer y enviar infomación a través de UART y mostrarla en la nube utilizando adafruit.io
//  
//
// Adafruit example written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"
#include <HardwareSerial.h>
#include <Ticker.h> //call a repeating function w/o using delays
#define LED 2   //Door open indicator LED
#define RXD2 16 // UART receive pin
#define TXD2 17 // UART transfer pin

Ticker timer;   //timer interruption
char zero = '0';  //send to UART to indicate door position
char two = '2';
boolean estado = false;   //door is closed by default
// variables for data
int temp, hum, gas, ired; //here we will save the sensor´s data

void handleMessage(AdafruitIO_Data *data);

// set up the 'proyecto1' feed
AdafruitIO_Feed *temp_canal = io.feed("Proyecto1_temp");
AdafruitIO_Feed *hum_canal = io.feed("Proyecto1_hum");
AdafruitIO_Feed *gas_canal = io.feed("Proyecto1_gas");
AdafruitIO_Feed *ir_canal = io.feed("Proyecto1_ir");
AdafruitIO_Feed *puerta_canal = io.feed("Proyecto1_puerta");

void onTimer()
{
  // save to the feed on Adafruit IO

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.

  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events. 
  io.run();
  Serial.print("sending -> ");
  Serial.println("datos");    //print data in terminal for testing
  temp_canal->save(temp);   //save values in each feed
  delay(2000);
  hum_canal->save(hum);
  delay(2000);
  gas_canal->save(gas);
  delay(2000);
  ir_canal->save(ired);

  Serial.println(temp); //print data in terminal for testing
  Serial.print("Humedad: ");
  Serial.println(hum);
  Serial.print("Nivel de Gas: ");
  Serial.println(gas);
  Serial.print("Estado de infrarrojo: ");
  Serial.println(ired); 

  if (estado)     //check slider for opening and closing door
  {
    digitalWrite(LED, HIGH);
    Serial2.print(two);   //send position via UART
    Serial.println(two);
  }
  else
  {
    digitalWrite(LED, LOW);
    Serial2.print(zero);
    Serial.println(zero);
  }
}

void setup()
{

  // start the serial connection
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  // wait for serial monitor to open
  while (!Serial)
    ;

  Serial.println();
  Serial.println("Program started");
  Serial.println("Connecting to Adafruit IO");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  estado = false;

  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the count feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  puerta_canal->onMessage(handleMessage);

  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  puerta_canal->get();
  timer.attach(10.0, onTimer); // call function onTimer every 10 seconds
}

void loop()
{

  if (Serial2.available())  // UART data received
  {
    // Read full serial line
    String receivedData = Serial2.readStringUntil('\n');

    // analize serial data
    int pos1 = receivedData.indexOf(' ');           // find position of the first space
    int pos2 = receivedData.indexOf(' ', pos1 + 1); // find position of the second space
    int pos3 = receivedData.indexOf(' ', pos2 + 1); // find position of the third space

    // extract data and convet to int
    if (pos1 != -1 && pos2 != -1 && pos3 != -1)
    {
      temp = receivedData.substring(0, pos1).toInt();
      hum = receivedData.substring(pos1 + 1, pos2).toInt();
      gas = receivedData.substring(pos2 + 1, pos3).toInt();
      ired = receivedData.substring(pos3 + 1).toInt();

    }
  }


  delay(100);
}

// this function is called whenever a feed message
// is received from Adafruit IO. it was attached to
// the feed in the setup() function above.
void handleMessage(AdafruitIO_Data *data)
{

  Serial.print("received <- ");
  Serial.println(data->value());
  if (*data->value() == '1')
  {
    estado = true;
  }
  else
  {
    estado = false;
  }
}
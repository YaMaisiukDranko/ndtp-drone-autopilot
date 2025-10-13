/*
Победа над nRF24L01: на три шага ближе, сканер эфира
https://habr.com/ru/post/476716/
*/

/*
  Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.
*/

/**
   Channel scanner

   Example to detect interference on the various channels available.
   This is a good diagnostic tool to check whether you're picking a
   good channel for your application.

   Inspired by cpixip.
   See http://arduino.cc/forum/index.php/topic,54795.0.html
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
#define CE_PIN    7   // Можно использовать любой GPIO
#define CSN_PIN   18
#define MOSI_PIN   19
#define MISO_PIN   20 
#define SCK_PIN   21 
RF24 radio(CE_PIN, CSN_PIN); //Arduino UNO

//
// Channel info
//

const uint8_t num_channels = 128;
uint8_t values[num_channels];

//
// Setup
//

void setup(void)
{
  //
  // Print preamble
  //
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  Serial.begin(9600);
  Serial.println("Scanner Air On");
  printf_begin();

  //
  // Setup and configure rf radio
  //

  radio.begin();
  radio.setAutoAck(false);

  // Get into standby mode
  radio.startListening();
  radio.printDetails();  
  delay(5000);              

  // Print out header, high then low digit
  int i = 0;
  while ( i < num_channels )
  {
    printf("%x", i >> 4);
    ++i;
  }
  printf("\n\r");
  i = 0;
  while ( i < num_channels )
  {
    printf("%x", i & 0xf);
    ++i;
  }
  printf("\n\r");
}

//
// Loop
//

const int num_reps = 100;

void loop(void)
{
  // Clear measurement values
  memset(values, 0, sizeof(values));

  // Scan all channels num_reps times
  int rep_counter = num_reps;
  while (rep_counter--)
  {
    int i = num_channels;
    while (i--)
    {
      // Select this channel
      radio.setChannel(i);

      // Listen for a little
      radio.startListening();
      delayMicroseconds(512);
      radio.stopListening();

      // Did we get a carrier?
      if ( radio.testCarrier() )
        ++values[i];
    }
  }

  // Print out channel measurements, clamped to a single hex digit
  int i = 0;
  while ( i < num_channels )
  {
    printf("%x", min(0xf, values[i] & 0xf));
    ++i;
  }
  printf("\n\r");
}
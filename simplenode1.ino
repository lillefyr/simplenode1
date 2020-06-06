/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
//#include <SPI.h>

//#include <Wire.h>

#include <SoftwareSerial.h>
#include <TinyGPS.h>

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

SoftwareSerial mySerial(8, 9);
TinyGPS gps;

void gpsdump(TinyGPS &gps);

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = { 0x9B, 0xBE, 0x57, 0x5F, 0x77, 0x1C, 0xE1, 0xDE, 0xF3, 0x17, 0x34, 0xF5, 0xC6, 0xF7, 0x12, 0x4F }; //simplenode1
//Old simplenode1 { 0x1C, 0x11, 0x86, 0xDF, 0xDD, 0x56, 0x0F, 0x65, 0x5C, 0x63, 0xD6, 0x03, 0xCE, 0x8B, 0xDB, 0x11 }; // simplenode1

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = { 0x2C, 0x0D, 0x20, 0x48, 0xDB, 0x0A, 0x14, 0x5D, 0x45, 0xE6, 0x41, 0x3E, 0xF1, 0x83, 0xA3, 0xD2 }; // simplenode1
//Old simplenode1 { 0xF0, 0x6E, 0x32, 0xAA, 0x1D, 0xFC, 0x58, 0xE8, 0x56, 0x9E, 0xE2, 0xA1, 0x63, 0x62, 0x6D, 0x00 }; //simplenode1

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR =  0x260118F8; // Old simplenode1 application 0x26011726; // simplenode1

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60; // every 1 minute (600) // can only run 5 hours pr day!

// Pin mapping 5V Arduino pro // simplenode1
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 6,
    .dio = {4, 5, LMIC_UNUSED_PIN},
};

u1_t myPort=1;

void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(F(": "));

    switch(ev) {

        case EV_TXCOMPLETE:

        /*
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            */
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;

         default:
            //Serial.println(F("Unknown event"));
            break;
    }
}

void getGPSData() {
  
  unsigned long start = millis();

  Serial.println(F("getGPSData"));
  
  do 
  {
    while (mySerial.available()) { gps.encode(mySerial.read()); }
  } while (millis() - start < 1000);
}

//static void print_float (float val, int len, int prec)
//  {
//    Serial.print(val, prec);
//    int vi = abs((int)val);
//    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
//    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//    for (int i=flen; i<len; ++i) Serial.print(' ');
//    Serial.print(' ');
//  }
  
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        uint8_t mydata[17];
        unsigned long int age, hdop, cnt;
        int year;
        byte month, day, hour, minute, second, hundredths;

        union u_tag2 {
            uint32_t i;
            float val;
        } flat,flon,falt,fcourse,fkmph;

        falt.val = 1000000.00;
        // Stay in loop until we get data. Blink once a minute
        while ( falt.val > 900000.00 ) {
          getGPSData();
          gps.f_get_position(&flat.val, &flon.val, &age);
          gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
          hdop = gps.hdop();
          falt.val = gps.f_altitude();
          delay(2000);                       // wait for a second
        }

// 6660199251651115175672500011919117618410

       if (( falt.val < 900000.00 ) && ( falt.val > -1000.00 )) {
         // pack date in an integer
        
         unsigned long int datetime = year - 2000;
         datetime = (datetime * 100) + month;
         datetime = (datetime * 100) + day;
         datetime = (datetime * 100) + hour;
         datetime = (datetime * 100) + minute;

//       if ( falt.val > 900000.00 ) {
//          flat.val = 47.195294;
//          flon.val = 8.724437;
//          falt.val = 500.0;
//          hdop = 10;
//          datetime = 1809051320;
//        }
        
          mydata[0] = flat.i >> 24;
          mydata[1] = flat.i >> 16;
          mydata[2] = flat.i >> 8;
          mydata[3] = flat.i;

          mydata[4] = flon.i >> 24;
          mydata[5] = flon.i >> 16;
          mydata[6] = flon.i >> 8;
          mydata[7] = flon.i;
        
          mydata[8] = falt.i >> 24;
          mydata[9] = falt.i >> 16;
          mydata[10] = falt.i >> 8;
          mydata[11] = falt.i;

          mydata[12] = datetime >> 24;
          mydata[13] = datetime >> 16;
          mydata[14] = datetime >> 8;
          mydata[15] = datetime;

          mydata[16] = hdop;

//        for (int i=0; i< sizeof(mydata); i++) { Serial.print(mydata[i]); }
//        Serial.println();

//        gpsdump( flat.val, flon.val, falt.val, fcourse.val, fkmph.val, age, datetime, hdop );
          
          LMIC_setTxData2(myPort, mydata, sizeof(mydata), 0);
          Serial.println(F("Packet queued"));

          //myPort++;
          //if (myPort > 3) { myPort=1; }
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//#define DUMPDATA 1
#ifdef DUMPDATA
void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print(F('-'));
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(F("."));

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

void gpsdump( float flat, float flon, float falt, float fcourse, float fkmph, unsigned long age, unsigned long datetime, unsigned long hdop )
{
  unsigned long chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

//  gps.get_position(&lat, &lon, &age);
//  Serial.print(F("Lat/Long(10^-5 deg): ")); Serial.print(lat); Serial.print(F(", ")); Serial.print(lon);
//  Serial.print(F(" Fix age: ")); Serial.print(age); Serial.println(F("ms."));

  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  Serial.print(F("Lat/Long(float): ")); printFloat(flat, 5);
  Serial.print(F(", ")); printFloat(flon, 5);
//  Serial.print(F(" Fix age: ")); Serial.print(age); Serial.print(F("ms. "));
  Serial.print(F(" (hdop): ")); printFloat(hdop); Serial.println();
  //Serial.print(F("Date(ddmmyy): ")); Serial.print(date); 
  //Serial.print(F(" Time(hhmmsscc): ")); 
  Serial.println(datetime);

/*
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  Serial.print(F("Date: ")); Serial.print(static_cast<int>(month)); Serial.print(F("/")); Serial.print(static_cast<int>(day)); Serial.print(F("/")); Serial.print(year);
  Serial.print(F("  Time: ")); Serial.print(static_cast<int>(hour+8));  Serial.print(F(":")); 
    Serial.print(static_cast<int>(minute)); Serial.print(F(":")); Serial.print(static_cast<int>(second));
    Serial.print(F(".")); Serial.print(static_cast<int>(hundredths)); Serial.print(F(" UTC +01:00 Switzerland"));
    */
    /*
  Serial.print(F("  Fix age: "));  Serial.print(age); Serial.println(F("ms."));
  */

 /*
  Serial.print(F("Alt(cm): ")); Serial.print(gps.altitude());
  Serial.print(F(" Course(10^-2 deg): ")); Serial.print(gps.course());
  Serial.print(F(" Speed(10^-2 knots): ")); Serial.println(gps.speed());
  */
  Serial.print(F("Alt(float): ")); printFloat(falt); 
/*  
  Serial.print(F(" Course(float): ")); printFloat(gps.f_course()); Serial.println();
  Serial.print(F("Speed(knots): ")); printFloat(gps.f_speed_knots()); 
  Serial.print(F(" (mph): ")); printFloat(gps.f_speed_mph());
  Serial.print(F(" (mps): ")); printFloat(gps.f_speed_mps()); 
  Serial.print(F(" (kmph): ")); printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print(F("Stats: characters: ")); Serial.print(chars); 
  Serial.print(F(" sentences: ")); Serial.print(sentences); 
  Serial.print(F(" failed checksum: ")); Serial.println(failed);
  */
}
#endif

void setup() {
    Serial.begin(115200);

    // set the data rate for the SoftwareSerial port
    mySerial.begin(9600);
    delay(1000);
    Serial.println(F("simplenode1"));
    Serial.println(F("uBlox Neo 6M"));
    Serial.print(F("Sizeof(gpsobject) = "));
    Serial.println(sizeof(TinyGPS));

    // initialize digital pin LED_BUILTIN as an output and turn off led.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // LMIC init
    //Serial.println(F("os_init"));
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    
    //Serial.println(F("LMIC_reset"));
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    LMIC_disableChannel(1);
    LMIC_disableChannel(2);
    LMIC_disableChannel(3);
    LMIC_disableChannel(4);
    LMIC_disableChannel(5);
    LMIC_disableChannel(6);
    LMIC_disableChannel(7);
    LMIC_disableChannel(8);
    
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

/*******************************************************************************
 * Copyright (c) 2016 Maarten Westenberg
 * based on work of Thomas Telkamp, Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This sketch sends a valid LoRaWAN packet with payload a DS18B 20 temperature 
 * sensor reading that will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 * History: 
 * 2017-02-14 rxf
 *   use an DHT22-Sensor instead of DALLAS
 *
 *  2017-01-29 rxf
 *    adopted, to use SDS011 Particulate Matter Sensor
 *	  Sends data every minute to LoRaWan
 * Jan 2016, Modified by Maarten to run on ESP8266. Running on Wemos D1-mini
 *
 *******************************************************************************/
 
// Use ESP declarations. This sketch does not use WiFi stack of ESP
#include <ESP8266WiFi.h>
#include <Esp.h>
#include <base64.h>

// All specific changes needed for ESP8266 need be made in hal.cpp if possible
// Include ESP environment definitions in lmic.h (lmic/limic.h) if needed
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Serial for SDS011
#include "SoftwareSerial.h"

//---------------------------------------------------------
// LoRaWAN settings (for thethingsnetwork)
//---------------------------------------------------------

// Time between transmissions to LoRa in sec 
#define LORA_SEND_TIME 60


// LoRaWAN Application identifier ^ ^(AppEUI)
// Not used in this example
static const u1_t APPEUI[8]  = {  };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8]  = {  };

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
static const u1_t DEVKEY[16] = {  };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
static const u1_t ARTKEY[16] = {  };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x0; // <-- Change this address for every node! ESP8266 node 0x01

// **********************************************************
// ******   Above settinge have to be adopted !!! ***********
// **********************************************************



//---------------------------------------------------------
// Sensor declarations
//---------------------------------------------------------
#define SDS011 1				// uses SDS011
#define S_DALLAS 0				// Use DS18B20 for temperature
#define S_DHT 1					// Use DHT22

#if S_DALLAS == 1
#define ONE_WIRE_BUS 5				// GPIO5 / D1  -> Data Pin of DS1820
#include "DallasTemperature.h"
  OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature sensors(&oneWire);
  int numberOfDevices; 					// Number of temperature devices found
#endif

#if S_DHT == 1
#include <DHT.h>
#define DHT_PIN D1
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
#endif

//---------------------------------------------------------
// APPLICATION CALLBACKS
//---------------------------------------------------------

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

#if SDS011 == 1
//---------------------------------------------------------
// div. timings for SDS011
//---------------------------------------------------------
#define SDS_SAMPLE_TIME 1000
#define SDS_WARMUP_TIME 10
#define SDS_READ_TIME 5
#endif
//---------------------------------------------------------
// Global Variables
//---------------------------------------------------------
int debug=1;
uint8_t mydata[64];
static osjob_t sendjob;

#if SDS011 == 1
// SDS-Variables
unsigned long act_milli, prev_milli;		// Timer-Ticks to calculate 1 sec
bool is_SDS_running = true;					// true, if SDS011 is running
uint8_t timer_SDS;							// Timer with 1sec ticks for SDS011 timimg

// Variables to calculate avereage for SDS011-Data
int sds_pm10_sum = 0;					
int sds_pm25_sum = 0;
int sds_val_count = 0;

// Kommands to start and stop SDS011
const byte stop_SDS_cmd[] = {0xFF, 0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
const byte start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
#endif

// JSON-Strings for the measurements of sensors
#if SDS011 == 1
String result_SDS = "";
byte result_SDS_by[4];
#endif
#if S_DALLAS == 1
String result_DALLAS = "";
byte result_DALLAS_by;
#endif
#if S_DHT == 1
String result_DHT = "";
byte result_DHT_by[2];
#endif


// Pin mapping for RFM95
// XXX We have to see whether all these pins are really used
// if not, we can use them for real sensor work.
lmic_pinmap pins = {
  .nss = 15,			// Make D8/GPIO15, is nSS on ESP8266
  .rxtx = 0xFF, 		// Not used
						// Do not connected on RFM92/RFM95
  .rst = 0xFF,  		// Not used
//  .dio = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
  .dio = {16, 4, 0xFF},	// Specify pin numbers for DIO0, 1, 2;  DIO2 not used
};
// Reset, DIO2 and RxTx are not connected AND in hal.cpp NOT initialised.
// So SDS011 can be connected:
// SDS011-RX <-> D4 and SADS011-TX <-> D3

#if SDS011 == 1
// Pinning for SDS011
// connected to SDS-TX
#define SDS_PIN_RX D3
// connected to SDS-RX
#define SDS_PIN_TX D4
SoftwareSerial serialSDS(SDS_PIN_RX, SDS_PIN_TX, false, 128);
#endif

void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          Serial.print("Event EV_TXCOMPLETE, time: ");
          Serial.println(millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println("Data Received!");
          }
          break;
       default:
          break;
    }
}

/*****************************************************************
/* convert value to json string extra for MQTT                   *
/*****************************************************************/
String Value2JsonMQTT(char* str, ...) {
  va_list(args);
  va_start(args, str);

  char * strArg;
  String s = "{";
  
  for (strArg = str; strArg != NULL; strArg = va_arg(args, char*)) {
      s += "\""+(String) strArg+"\"";
      s += ":";
      strArg = va_arg(args, char*);
      s += "\""+(String)strArg+"\"";
      s += ",";
  }
  s = s.substring(0, s.length() - 1);
  s += "}";
  return s;
}

/*****************************************************************
/* convert float to string with a                                *
/* precision of 1 decimal place                                  *
/*****************************************************************/
String Float2String(const float value) {
	// Convert a float to String with two decimals.
	char temp[15];
	String s;

	dtostrf(value,13, 1, temp);
	s = String(temp);
	s.trim();
	return s;
}

#if SDS011
/*****************************************************************
/* read SDS011 sensor values                                     *
/*****************************************************************/
void sensorSDS() {
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is;
	int checksum_ok = 0;

	if (! is_SDS_running) {
		return;
	}
	
	// SDS runs: read serial buffer
	while (serialSDS.available() > 0) {
		buffer = serialSDS.read();
//			Serial.println(String(len)+" - "+String(buffer,DEC)+" - "+String(buffer,HEX)+" - "+int(buffer)+" .");
//			"aa" = 170, "ab" = 171, "c0" = 192
		value = int(buffer);
		switch (len) {
			case (0): if (value != 170) { len = -1; }; break;
			case (1): if (value != 192) { len = -1; }; break;
			case (2): pm25_serial = value; checksum_is = value; break;
			case (3): pm25_serial += (value << 8); checksum_is += value; break;
			case (4): pm10_serial = value; checksum_is += value; break;
			case (5): pm10_serial += (value << 8); checksum_is += value; break;
			case (6): checksum_is += value; break;
			case (7): checksum_is += value; break;
			case (8):
//					  Serial.println("Checksum is: "+String(checksum_is % 256)+" - should: "+String(value));
					  if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
			case (9): if (value != 171) { len = -1; }; break;
		}
		len++;
		if ((len == 10 && checksum_ok == 1) && (timer_SDS > SDS_WARMUP_TIME)) {
			if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
				sds_pm10_sum += pm10_serial;
				sds_pm25_sum += pm25_serial;
				sds_val_count++;
			}
			len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
		}
		yield();
	}

	// Data for SDS_READTIME time is read: now calculate the average and return value
	if (timer_SDS > (SDS_WARMUP_TIME + SDS_READ_TIME)) {
		// Calculate average
//		Serial.println("Sum: " + String(sds_pm10_sum) + "  Cnt: " + String(sds_val_count));
		String sp1_av = Float2String(float(sds_pm10_sum)/(sds_val_count*10.0));
		String sp2_av = Float2String(float(sds_pm25_sum)/(sds_val_count*10.0));
		Serial.println("PM10:  "+sp1_av);
		Serial.println("PM2.5: "+sp2_av);
		Serial.println("------");
		result_SDS = Value2JsonMQTT("P1",sp1_av.c_str(),"P2",sp2_av.c_str(),NULL);
    int sdsp1 = (int)(sds_pm10_sum/sds_val_count);
    int sdsp2 = (int)(sds_pm25_sum/sds_val_count);
    result_SDS_by[0] = sdsp1>>8;
    result_SDS_by[1] = sdsp1&0xFF;
    result_SDS_by[2] = sdsp2>>8;
    result_SDS_by[3] = sdsp2&0xFF;
		// clear sums and count
		sds_pm10_sum = 0; sds_pm25_sum = 0; sds_val_count = 0;
		// and STOP SDS
		serialSDS.write(stop_SDS_cmd,sizeof(stop_SDS_cmd));	
		is_SDS_running = false;
		Serial.println("SDS stopped");
	}
}

#endif

#if S_DALLAS==1
/*****************************************************************
/* DALLAS-Sensor auslesen					                     *
/*****************************************************************/

// Dallas sensors (can be more than 1) have channel codes 3 and above!
void sensorDallas() {
	
	  uint8_t ind;
	  uint8_t erg[20];

        Serial.print("Search Dallas");
	  DeviceAddress tempDeviceAddress; 			// We'll use this variable to store a found device address
	  sensors.requestTemperatures();
	  for(int i=0; i<numberOfDevices; i++)
	  {
		// Search the wire for address
		if(sensors.getAddress(tempDeviceAddress, i)) {
			float tempC = sensors.getTempC(tempDeviceAddress);
			// Output the device ID
			if (debug>=1) {
				Serial.print(F("! DS18B20 dev ("));
				Serial.print(i);
			}
			int ival = (int) tempC;					// Make integer part
			int fval = (int) ((tempC - ival)*10);	// Fraction. Has same sign as integer part
			if (fval<0) fval = -fval;				// So if it is negative make fraction positive again.
			result_DALLAS = String("{\"T\":\"" + String(ival) + "." + String(fval)+"\"}");
      result_DALLAS_by = (byte)(tempC*2);
			if (debug>=1) {
				Serial.print(") ");
				Serial.println(result_DALLAS);
			}
	  } 
	 //else ghost device! Check your power requirements and cabling
	}
}
#endif	  
	  
#if S_DHT == 1
/*****************************************************************
/* read DHT22 sensor values                                      *
/*****************************************************************/
void sensorDHT() {
	float h = dht.readHumidity(); //Read Humidity
	float t = dht.readTemperature(); //Read Temperature

	Serial.println("Reading DHT22");

	// Check if valid number if non NaN (not a number) will be send.
	if (isnan(t) || isnan(h)) {
		Serial.println("DHT22 couldn't be read");
	} else {
		Serial.println("Humidity    : "+String(h)+"%");
		Serial.println("Temperature : "+String(t)+" C");
	}
	Serial.println("------");

  String st = Float2String(t);
  String sh = Float2String(h);
  result_DHT = Value2JsonMQTT("T",st.c_str(),"H",sh.c_str(),NULL);
  result_DHT_by[0] = (byte)(t*2);
  result_DHT_by[1] = (byte)h ;
}

#endif


// ----------------------------------------------------
// This function prepares a message for the LoRaWAN network
// The message will be sent multiple times.
//
void do_send(osjob_t* j){
	  Serial.println();
      Serial.print("Time: "); Serial.println(millis() / 1000);
      // Show TX channel (channel numbers are local to LMIC)
      Serial.print("Send, txCnhl: "); Serial.println(LMIC.txChnl);
      Serial.print("Opmode check: ");
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      Serial.println("OP_TXRXPEND, not sending");
    } else {
      Serial.print("ok, ready to send: ");
	  Serial.print((char *)mydata);
	  Serial.println();
	  
#if S_DALLAS == 1
// Read the sensor, store result in result_DALLAS
	  sensorDallas();
#endif
#if S_DHT == 1
	sensorDHT();
#endif

	  String tosend = "EMPTY";
    byte bytsend[10];                   // !!!! MAx 10 Bytes to send !!!!
    int idx = 0;
	// Build JSON-String to send to LoRa
#if SDS011==1
	  tosend = result_SDS;
    for (; idx<4; idx++) {
      bytsend[idx] = result_SDS_by[idx];
    }
    
#if S_DALLAS == 1
	  tosend.replace('}',',');
    result_DALLAS.replace('{',' ');
    tosend += result_DALLAS;
    bytsend[idx] = result_DALLAS_by;    
    idx++;
#endif	

#if S_DHT == 1
	  tosend.replace('}',',');
      result_DHT.replace('{',' ');
      tosend += result_DHT;
      bytsend[idx] = result_DHT_by[0];
      bytsend[idx+1] = result_DHT_by[1];
      idx+=2;
#endif	
      Serial.println("Sende: " + tosend);
#else
#if S_DALLAS == 1
	  tosend = result_DALLAS;
    bytsend[0] = result_DALLAS_by;
    idx = 1;
#endif
#if S_DHT == 1
	  tosend = result_DHT;
    bytsend[0] = result_DHT_by[0];
    bytsend[1] = result_DHT_by[1];
    idx = 2;
#endif
#endif

	  // prepare message 
	//  sprintf((char *)mydata,"%s",tosend.c_str());
    Serial.print("Länge ByteArray:");
    Serial.println(idx);

    
    memcpy((char *)mydata, (char *)bytsend, idx);
    int k;
    for(k=0; k<idx; k++) {
      Serial.print(mydata[k],HEX);
      Serial.print(" ");
    }
    Serial.println();
	  
      // Prepare upstream data transmission at the next possible time.
//      LMIC_setTxData2(1, mydata, strlen((char *)mydata), 0);
      LMIC_setTxData2(1, mydata, idx, 0);
    }
    
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime()+sec2osticks(LORA_SEND_TIME), do_send);
         
#if SDS011 == 1
    // Now start SDS senor
    serialSDS.write(start_SDS_cmd,sizeof(start_SDS_cmd)); 
	is_SDS_running = true;
	timer_SDS = 0;							// start timer
	Serial.println("SDS started");
#endif
}


// ----------------------------------------------------
// Remove the Serial messages once the unit is running reliable
// 
void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  // switch WiFi OFF
  WiFi.disconnect();
  WiFi.forceSleepBegin();
  delay(1); 

#if S_DALLAS==1
	sensors.begin();
	numberOfDevices = sensors.getDeviceCount();
	if (debug>=1) {
		Serial.print("DALLAS #:");
		Serial.print(numberOfDevices); 
		Serial.println(" ");
	}
#endif

  // LMIC init
  os_init();
  Serial.println("os_init() finished");
  
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  Serial.println("LMIC_reet() finished");
  
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, (uint8_t*)DEVKEY, (uint8_t*)ARTKEY);
  Serial.println("LMIC_setSession() finished");
  
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  Serial.println("LMICsetAddrMode() finished");
  
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  //
  Serial.println("Init done");

}


//---------------------------------------------------------
// main loop
// Loop is simple: read sensor value and send it to the LoRaWAN
// network.
//---------------------------------------------------------

void loop() {
	Serial.println("loop: Starting");
	strcpy((char *) mydata,"Starting ESP8266 Dallas\n");

// The do_send function puts a message in the queue and then puts
// itself to sleep. When waking up, will again work on queue again.

	do_send(&sendjob);						// Put job in run queue(send mydata buffer)
	delay(10);
	
	while(1) {
#if SDS011 == 1		
		act_milli = millis();				// read system-tick

		if((act_milli - prev_milli) >= SDS_SAMPLE_TIME) {   // after SAMPLE_TIME (==0 1sec)
			prev_milli = act_milli;
			timer_SDS += 1;					// Count SDS-Timer
			sensorSDS();					// check (and read)  SDS011		
		}
#endif
		os_runloop_once();					// Let the server run its jobs
		delay(100);
	}
	
}



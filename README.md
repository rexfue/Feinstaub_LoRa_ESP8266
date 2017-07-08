# Feinstaub\_LoRa\_ESP8266
Measurement of partriculatre matter using an ESP8266 with SDS011 and sending via LoRa to TheThingsNet

This project is based on the hard- and software of open-data-stuttgart for the measuremnt of paticulate matter. Instead of WiFi I use the RFM95 board to send the data via LoRa to the The Things Network (TTN).

###This project is only to show that it works. There is still much room for improvement !###


### Hardware
 
Interconnection between nodemcu, RFM95, SDS011 and DHT22:

NODEMCU PIN    |    goes to
---------------|-----------
D0  | RFM95 - DIO0
D1  | DHT22
D2  | RFM95 - DIO1
D3  | SDS011 - TX  (= nodemcu rx) 
D4  | SDS011 - RX  (= nodemcu tx)
D5  | RFM95 - SLCK
D6  | RFM95 - MISO
D7  | RFM95 - MOSI
D8  | RFM95 - NSS

Because of the limited IO pins of the nodemcu, there is only one pin for temperature and humidity measuremant (pin D1). I use the DHT22 for this.

###Software

The original lmic-1.51 library has to be changed a little bit. The only changes are in hal/hal.cpp. They are in the upper part of the file and marked with **//rxf** or **/\*rxf**. 

The reasons for this changes are: 

* DIO2 isn't used for LoRa,
* there is no rxtx pin on RFM95
* and reset will be generated at power on.

I put the adopted lmic library onto github into the library folder. Other necessary libraries can be downloaded as usual via arduino ide. 

To save power, WiFi is switched off. Repeat time for sending is 1 minute (LORA\_SEND\_TIME).

The personal credentials for TTN are set directly in the source. Find 'LoRaWAN settings' and fill in your data betwenn the empty braces.

### Data
To save airtime, the data is sent as bytes. The 6 bytes are:

ByteNr | Meaning
-------|---------
0  |  MSB of P10 * 10
1  |  LSB of P10 * 10  [ µg/m3 ]
2  |  MSB of P2.5 * 10
3  |  LSB of P2.5 * 10 [ µg/m3 ]
4  |  Temperature * 2 [ °C ]
5  |  Humidity [ % ]


The data only goes to TTN, no forwarding to open-data servers.

Payload function to decode the data on TTN:

    function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
       data = {
       "P1" : (bytes[0]*256+bytes[1])/10,
       "P2" : (bytes[2]*256+bytes[3])/10,
       "T": bytes[4]/2,
       "H": bytes[5]
       }
    return data;
    }



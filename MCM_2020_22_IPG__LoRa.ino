
#include "DHT.h"
//#include <DHT_U.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//#include <DHT.h>
#include <TinyGPS.h>

#include <SoftwareSerial.h>
//#define CHANNEL  0
SoftwareSerial ss(3, 4); // Arduino TX, RX ,

//DHT dht ;
#define DHT11_PIN A0
#define PIN_A A1
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT11   // DHT 11

//TinyGPS gps;
// GPS
float flat, flon;
unsigned long age, date, time, chars = 0;
unsigned short sentences = 0, failed = 0;

// The TinyGPS++ object
typedef union {
  float f[2];
  unsigned char bytes[8];
} floatArr2Val;
floatArr2Val latlong;


#define CHANNEL 1

float temperature,humidity;      
float tem,hum;

unsigned int count = 1;        //For times count

String datastring1="";        
String datastring2="";        
String datastring3="";


static uint8_t payload[7]; // reserve 5 bytes in memory



static const PROGMEM u1_t NWKSKEY[16] = { 0x2f , 0x4e , 0x09 , 0x84 , 0x41 , 0xf9 , 0x7b , 0x38 , 0x29 , 0x5a , 0x64 , 0x6d , 0x6e , 0x41 , 0x2e , 0x43};  //COLOCAR AQUI os secrets
static const u1_t PROGMEM APPSKEY[16] = { 0x66 , 0xa4 , 0x56 , 0xc4 , 0x10 , 0xe4 , 0xf3 , 0x22 , 0x3f , 0x79 , 0xe9 , 0xbd , 0x59 , 0x50 , 0xb7 , 0x48}; //COLOCAR AQUI os secrets
static const u4_t DEVADDR = 0x0154834c; // <-- Change this address for every node!

/*
static const PROGMEM u1_t NWKSKEY[16] = { 0x58 , 0xc7 , 0xb8 , 0x8d , 0x4f , 0x38 , 0xe9 , 0x09 , 0x6b , 0x86 , 0xd1 , 0xfa , 0x10 , 0xa1 , 0x0d , 0x5c};  //COLOCAR AQUI os secrets
static const u1_t PROGMEM APPSKEY[16] = { 0xc5 , 0x3a , 0x11 , 0xb4 , 0xd0 , 0x01 , 0xde , 0x29 , 0x94 , 0x72 , 0xc9 , 0x6c , 0xa3 , 0x18 , 0xe1 , 0x91}; //COLOCAR AQUI os secrets
static const u4_t DEVADDR = 0x26011874; // <-- Change this address for every node!

static const PROGMEM u1_t NWKSKEY[16] = { 0x2f , 0x4e , 0x09 , 0x84 , 0x41 , 0xf9 , 0x7b , 0x38 , 0x29 , 0x5a , 0x64 , 0x6d , 0x6e , 0x41 , 0x2e , 0x43};  //COLOCAR AQUI os secrets
static const u1_t PROGMEM APPSKEY[16] = { 0x66 , 0xa4 , 0x56 , 0xc4 , 0x10 , 0xe4 , 0xf3 , 0x22 , 0x3f , 0x79 , 0xe9 , 0xbd , 0x59 , 0x50 , 0xb7 , 0x48}; //COLOCAR AQUI os secrets
static const u4_t DEVADDR = 0x0154834c; // <-- Change this address for every node!

static const PROGMEM u1_t NWKSKEY[16] = { 0x20 , 0xf8 , 0x97 , 0x07 , 0x10 , 0x38 , 0x5c , 0x67 , 0x77 , 0x68 , 0x12 , 0x3f , 0xb1 , 0xa9 , 0x52 , 0x85};  //COLOCAR AQUI os secrets
static const u1_t PROGMEM APPSKEY[16] = { 0x70 , 0xa4 , 0x28 , 0xe0 , 0xcd , 0x83 , 0x33 , 0x71 , 0x47 , 0xc7 , 0x88 , 0xf9 , 0x52 , 0x92 , 0xf3 , 0xb9}; //COLOCAR AQUI os secrets
static const u4_t DEVADDR = 0x0149f388; // <-- Change this address for every node!
*/

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
//static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

/* These callbacks are only used in over-the-air activation, so they are
  left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain).*/
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t initjob,sendjob,blinkjob;

/* Schedule TX every this many seconds (might become longer due to duty
 cycle limitations).*/
const unsigned TX_INTERVAL = 10;

DHT dht(DHT11_PIN, DHTTYPE);

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};



void dhtTem()
{
       int16_t tem1;
       
        delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

       count++;


    int16_t temperatura = 100 * t;
    int16_t humidade = 100* h;
    


    // Handle high byte (MSB) first; 0xFF for -234 or 0x00 for +234
    // 0xFF16 >> 8 shifts the 0x16 out of memory, leaving 0x00FF
    // 0x00FF does not fit in a single byte, so only 0xFF is stored in buff[0]:
    payload[0] = temperatura >> 8;
    // Handle low byte (LSB) next; 0x16 for -234 or 0xEA for +234
    // 0xFF16 does not fit in a single byte, so only 0x16 is stored in buff[1]:
    payload[1] = temperatura;

    //mesma coisa aqui
    payload[2] = humidade >> 8;
    payload[3] = humidade;

}

void light(){
      int16_t lux;

      int val,val1;
      val=analogRead(PIN_A);
     Serial.print(F("a:"));
      Serial.println(val);
      delay(500);
      val1=val*1.0;
      lux=val1;

      payload[4] = lux >> 8;
      payload[5] = lux;

}
// ----------------------------- Imprimir CENAS -------------------------------------------------------------------------
void printPayload() {
  Serial.println("----- PAYLOAD ------");
  Serial.print("LatLon: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(payload[i], HEX);
  }
  Serial.println("");

  Serial.print("Lux: ");
  for (int i = 8; i < 10; i++) {
    Serial.print(payload[i], HEX);
  }
  Serial.println("");

  Serial.print("TemHum: ");
  for (int i = 10; i < 14; i++) {
    Serial.print(payload[i], HEX);
  }
  Serial.println("");
  Serial.println("------ PAYLOAD ------ ");
  for (int i = 0; i < 17; i++) {
    Serial.print(payload[i], HEX);
  }
  Serial.println("");
  Serial.println("------ PAYLOAD ------ ");
  Serial.println("");
}


void gpsValue() {
  float latitude;
  float longitude;
  //  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  // gps.f_get_position(&flat, &flon, &age);
  //  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 9);
  //  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 9);
  Serial.println();
  smartdelay(2000);

  latitude  = flat;
  longitude = flon;

  if ((latitude && longitude) && latitude != latlong.f[0]
      && longitude != latlong.f[1]) {
    latlong.f[0] = latitude;
    latlong.f[1] = longitude;

    Serial.print("LatLong: ");
    for (int i = 0; i < 8; i++) {
      //  Serial.print(latlong.bytes[i], HEX);
      payload[i] = latlong.bytes[i];
    }
    Serial.println("");
  }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    printPayload();
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else {
         gpsValue();
        dhtTem();
        light();
        // Prepare upstream data transmission at the next possible time.
        //  LMIC_setTxData2(1,datasend,sizeof(datasend)-1,0);
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println("Packet queued");
        Serial.print("LMIC.freq:");
        Serial.println(LMIC.freq);
        Serial.println("Receive data:");
      
        
    } 
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(ev);
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}



void setup() {
     // initialize digital pin  as an output.
   
    Serial.begin(9600);
     Serial.println("Connect to TTN and Send data to mydevice(Use DHT11 Sensor):");
    dht.begin();
    while(!Serial);
   
   
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    /*LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
     Set static session parameters. Instead of dynamically establishing a session
     by joining the network, precomputed session parameters are be provided.*/
    #ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again.*/
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    

    // DAEY single CHANNEL gateway
        #define CHANNEL  1

        for (uint8_t i = 0; i < 9; i++) {
            if (i != CHANNEL) {
                LMIC_disableChannel(i);
            }
        }

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
   // LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,20);

    // Start job
    do_send(&sendjob);
}


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
    {
      ss.print(Serial.read());
      //gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

/*static void print_date(TinyGPS &gps)
  {
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
  }*/

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartdelay(0);
}



void loop() {
    os_runloop_once();
       
}

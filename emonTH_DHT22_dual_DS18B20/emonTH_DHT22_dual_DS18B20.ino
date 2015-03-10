/*
 emonTH low power temperature & humidity node
 ============================================
 
 Ambient humidity & temperature (DHT22 on-board)
 Multiple remote temperature (DS18B20)
 
 Provides the following inputs to emonCMS:
 
 1. Battery voltage
 2. Humidity (with DHT22 on-board sensor, otherwise zero)
 3. Ambient temperature (with DHT22 on-board sensor, otherwise zero)
 4. External temperature 1 (first external DS18B20)
 5. External temperature 2 (second external DS18B20)
 
 Easily extended to support further external DS18B20 sensors, see comments below.
 
 -----------------------------------------------------------------------------------------------------------  
 Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
 Part of the openenergymonitor.org project
 Licence: GNU GPL V3
 
 Authors: Dave McCraw,
 
 Based on the emonTH_DHT22_DS18B20 sketch by Glyn Hudson.
 
 THIS SKETCH REQUIRES:
 
 Libraries in the standard arduino libraries folder:
   - RFu JeeLib           https://github.com/openenergymonitor/RFu_jeelib   - to work with CISECO RFu328 module
   - DHT22 Sensor Library https://github.com/adafruit/DHT-sensor-library    - be sure to rename the sketch folder to remove the '-'
   - OneWire library      http://www.pjrc.com/teensy/td_libs_OneWire.html   - DS18B20 sensors
   - DallasTemperature    http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip - DS18B20 sensors
 */
#include <avr/power.h>
#include <avr/sleep.h>
#include <RFu_JeeLib.h>                                                 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RFM69.h>
#include <SPI.h>
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // Attached JeeLib sleep function to Atmega328 watchdog - enables MCU to be put into sleep mode between readings to reduce power consumption 

 
/*
 Network configuration
 =====================
*/
#define GATEWAY_ID    1
#define NODE_ID       21    // node ID
#define NETWORKID     100    //the same on all nodes that talk to each other
#define MSG_INTERVAL  30000  // 30000=30s

// Uncomment only one of the following three to match radio frequency
//#define FREQUENCY     RF69_433MHZ    
#define FREQUENCY     RF69_868MHZ
//efine FREQUENCY     RF69_915MHZ

#define IS_RFM69HW   //NOTE: uncomment this ONLY for RFM69HW or RFM69HCW
#define ENCRYPT_KEY    "0123456789012345"  // use same 16byte encryption key for all devices on net
#define ACK_TIME       50                  // max msec for ACK wait
#define LED            9                   // Anardino miniWireless has LEDs on D9
#define SERIAL_BAUD    115200
#define TNODE_VERSION  "1.0"

/*
 Monitoring configuration
 ========================
 
  - how long to wait between readings, in minutes
  - DS18B20 temperature precision:
      9bit: 0.5C,  10bit: 0.25C,  11bit: 0.1125C, 12bit: 0.0625C
  - Required delay when reading DS18B20
      9bit: 95ms,  10bit: 187ms,  11bit: 375ms,   12bit: 750ms
 */
const int MINS_BETWEEN_READINGS = 5; // minutes between readings
const int TEMPERATURE_PRECISION = 11; 
const int ASYNC_DELAY           = 375;

// emonTH pin allocations 
const int BATT_ADC     = 1; // will use vcc instead
const int DS18B20_PWR  = 5;
const int LED          = 9;
const int ONE_WIRE_BUS = 4;  

// OneWire for DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* THIS MIGHT NOT NEEDED HERE
union itag {
  uint8_t b[2];
  uint16_t i;
}it;
union ltag {
  byte b[4];
  long l;
}lt; // used to force byte order in case we end up using result in various endian targets...
*/

// RFM69 RF payload datastructure
typedef struct {
  uint8_t          nodeId;
  unsigned long    uptime;               //4 byte
  int8_t           tempC;                //1 bytes from RFM69
  int16_t          vcc;                  //2 bytes in miliVolts
  int16_t          extTempC1;          //2 bytes in centigrades
  int16_t          extTempC2;          //2 bytes in centigrades
} Payload;

Payload rfPayload;

RFM69 radio;                  // global radio instance
bool promiscuousMode = false; // set 'true' to sniff all packets on the same network
bool requestACK=false;

boolean debug;

/*
  External sensor addresses
  =========================
  
 Hardcoding these guarantees emonCMS inputs won't flip around if you replace or add sensors.
 Use one of the address finding sketches to determine your sensors' unique addresses.

 See 'emonTH temperature search' utility sketch in 'Simple emonTH Sensor Test' folder
 
 Extend this if you have more sensors.
 */
DeviceAddress EXT_SENSOR1 = { 
  0x28, 0x46, 0x59, 0x44, 0x05, 0x00, 0x00, 0x64 };
DeviceAddress EXT_SENSOR2 = { 
  0x28, 0x9B, 0x24, 0x44, 0x05, 0x00, 0x00, 0x6B };

boolean EXT_SENSOR1_PRESENT;  
boolean EXT_SENSOR2_PRESENT;  

/**
 * setup() - called once on boot to initialise the emonTH
 */
void setup() {

  // Output only if serial UART to USB is connected
  debug = Serial ? 1 : 0;                              

  print_welcome_message();  
  set_pin_modes();

  // LED on
  digitalWrite(LED, HIGH);

  // Initalize RFM69
  radio.initialize(FREQUENCY, NODE_ID, NETWORKID);
  radio.encrypt(0);
  radio.promiscuous(promiscuousMode);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment #define ONLY if radio is of type: RFM69HW or RFM69HCW 
#endif
  
  reduce_power();
  
  // Initialise sensors
  initialise_DS18B20();

  // Confirm we've got at least one sensor, or shut down.
  validate_sensor_presence();

  // LED off
  digitalWrite(LED, LOW);

} // end of setup

/**
 * Perform temperature and humidity logging
 */
void loop()
{ 
  // External temperature readings
  if (EXT_SENSOR1_PRESENT || EXT_SENSOR2_PRESENT) 
    take_ds18b20_reading();

  // Battery reading
  take_battery_reading();

  // Debugging
  print_payload();                                             

  power_spi_enable();    
  radio.sendWithRetry((byte)GATEWAY_ID, (const void *)(&myPayload),sizeof(myPayload), 2);
  radio.sleep(); // ??
  power_spi_disable();  

  if (debug){
    flash_led(50);
  }

  // That's it - wait until next time :)
  sleep_until_next_reading();
}



/**
 * Turn off what we don't need.
 * see http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
 */
void reduce_power()
{
  ACSR |= (1 << ACD);              // Disable Analog comparator    
  power_twi_disable();             // Disable the Two Wire Interface module.

  power_timer1_disable();          // Timer 1
  power_spi_disable();             // Serial peripheral interface

    if (!debug){
    power_usart0_disable();        // Disable serial UART if not connected
  }  

  power_timer0_enable();           // Necessary for the DS18B20 library.
}

/**
 * Set the pin modes required for this sketch
 */
void set_pin_modes()
{
  pinMode(LED,         OUTPUT); 
  pinMode(BATT_ADC,    INPUT);
}

/**
 * Find the expected DS18B20 sensors
 *
 * You will need your sensors' unique address (obtain using one of the sketches designed for this purpose).
 * 
 * Straightforward to add support for additional sensors by extending this function.
 */
void initialise_DS18B20()
{
  // Switch on
  digitalWrite(DS18B20_PWR, HIGH); 
  dodelay(50); 

  sensors.begin();

  // Disable automatic temperature conversion to reduce time spent awake, instead we sleep for ASYNC_DELAY
  // see http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/ 
  sensors.setWaitForConversion(false);                             

  // Note - the index (param 2) is dependant on the natural ordering of your sensor addresses.
  // We'll try both ways, just in case.
  EXT_SENSOR1_PRESENT = sensors.getAddress(EXT_SENSOR1, 0);
  EXT_SENSOR2_PRESENT = sensors.getAddress(EXT_SENSOR2, 1);

  EXT_SENSOR1_PRESENT = EXT_SENSOR1_PRESENT ? EXT_SENSOR1_PRESENT : sensors.getAddress(EXT_SENSOR1, 1);
  EXT_SENSOR2_PRESENT = EXT_SENSOR2_PRESENT ? EXT_SENSOR2_PRESENT : sensors.getAddress(EXT_SENSOR2, 0);
  // .. for 3 or more sensors you may want to establish the order by trial and error instead

  // No luck?
  if (debug){
    
    if (!EXT_SENSOR1_PRESENT) 
      Serial.println("Unable to find address for DS18B20 External 1... check hard coded address");
    else 
      Serial.println("Found DS18B20 External 1");
      
    
    if (!EXT_SENSOR2_PRESENT) 
      Serial.println("Unable to find address for DS18B20 External 2... check hard coded address");
    else 
      Serial.println("Found DS18B20 External 2");
      
    // .. and for sensor 3, etc...
  }

  // Switch off for now
  digitalWrite(DS18B20_PWR, LOW);
}


/** 
 * If we don't have at least one sensor available, it's sleep time
 */
void validate_sensor_presence()
{
  if (!EXT_SENSOR1_PRESENT && !EXT_SENSOR2_PRESENT) 
  {
    if (debug) {
      
      Serial.print("Power down - no sensors detected at all!");
    }
    
    for (int i=0; i<10; i++)
    {
      flash_led(250); 
      dodelay(250);
    }
    cli();                                      //stop responding to interrupts 
    Sleepy::powerDown();                        //sleep forever
  }
}


/**
 * Convenience method; battery reading
 */
void take_battery_reading()
{
  // convert ADC to volts x10
  //rfPayload.battery=int(analogRead(BATT_ADC)*0.03225806);                 
  // in millivolts
  rfPayload.battery=readVcc();
}

int readVcc() {   // return vcc voltage in millivolts
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate in mV
  int t = (int)result;
  return t;
}


/**
 * Convenience method; read from all DS18B20s
 * You will need to extend this to read from any extra sensors.
 */
void take_ds18b20_reading () 
{
  // Power up
  digitalWrite(DS18B20_PWR, HIGH); 
  dodelay(50); 

  // Set precision to desired value
  sensors.setResolution(EXT_SENSOR1, TEMPERATURE_PRECISION);
  sensors.setResolution(EXT_SENSOR2, TEMPERATURE_PRECISION);

  // Get readings. We must wait for ASYNC_DELAY due to power-saving (waitForConversion = false)
  sensors.requestTemperatures();                                   
  dodelay(ASYNC_DELAY); 
  float temp1=(sensors.getTempC(EXT_SENSOR1));
  float temp2=(sensors.getTempC(EXT_SENSOR2)); 

  // Power down
  digitalWrite(DS18B20_PWR, LOW);

  // Payload will maintain previous reading unless the temperature is within range.
  if (temperature_in_range(temp1))
    rfPayload.externalTemp1= temp1 * 10;   

  if (temperature_in_range(temp2))
    rfPayload.externalTemp2= temp2 * 10;   
}

/**
 * To save power, we go to sleep between readings
 */
void sleep_until_next_reading(){
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
  
  //for (int i=0; i<MINS_BETWEEN_READINGS; i++) {
  //    Sleepy::loseSomeTime(55000);  
  //}
  Sleepy::loseSomeTime(30000);
  
  ADCSRA=oldADCSRA; // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

/**
 * For debugging purposes: print the payload as it will shortly be sent to the emonBASE
 * Wise to extend this if you have extra sensors wired in.
 */
void print_payload()
{
  if (!debug)
    return;
  
  Serial.println("emonTH payload: ");

  Serial.print("  Battery voltage: ");
  Serial.print(rfPayload.battery/1000.0);
  Serial.println("V");
  
  Serial.print("  External sensor 1: ");

  if (EXT_SENSOR1_PRESENT){
    Serial.print( rfPayload.externalTemp1/10.0); 
    Serial.println("C");
  }
  else {
    Serial.println(" not present");
  }
    
  Serial.print("  External sensor 2: ");

  if (EXT_SENSOR2_PRESENT){
    Serial.print( rfPayload.externalTemp2/10.0); 
    Serial.println("C");
  }
  else {
    Serial.println(" not present");
  }
  
  
  Serial.println();
}


/**
 * Dumps useful intro to serial
 */
void print_welcome_message()
{
  if (!debug)
    return;

  Serial.begin(SERIAL_BAUD);

  Serial.println("emonTH : OpenEnergyMonitor.org");
  
  Serial.print("Node: "); 
  Serial.print(NODE_ID); 
 
  Serial.print(" Freq: "); 
  switch(FREQUENCY){
  case RF12_433MHZ:
    Serial.print("433Mhz");
    break;
  case RF12_868MHZ:
    Serial.print("868Mhz");
    break;
  case RF12_915MHZ:
    Serial.print("915Mhz");
    break;
  }

  
  Serial.print(" Network: "); 
  Serial.println(NETWORK_GROUP);

  
  dodelay(100);
}


/**
 * validate that the provided temperature is within acceptable bounds.
 */
boolean temperature_in_range(float temp)
{
  // Only accept the reading if it's within a desired range.
  float minimumTemp = -40.0;
  float maximumTemp = 125.0;

  return temp > minimumTemp && temp < maximumTemp;
}


/**
 * Flash the LED for the stated period
 */
void flash_led (int duration){
  digitalWrite(LED,HIGH);
  dodelay(duration);
  digitalWrite(LED,LOW); 
}


/**
 * Power-friendly delay
 */
void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;

  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)

  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}


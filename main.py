#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"
#include "TSL2561.h"
#include "Timer.h"

#ifdef DEBUG 
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x) 
#endif

// temperature humidity sensor
#define DHTPIN 5          // what digital pin we're connected to
#define DHTTYPE DHT22     // DHT 22
DHT dht(DHTPIN, DHTTYPE);

// light sensor
TSL2561 tsl(TSL2561_ADDR_FLOAT); 

// OLED display buttons
Adafruit_SSD1306 display = Adafruit_SSD1306();
#if defined(ESP8266)
  #define BUTTON_A 0
  #define BUTTON_B 16
  #define BUTTON_C 2
  #define LED      0
#elif defined(ARDUINO_STM32F2_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
  #define LED PB5
#elif defined(TEENSYDUINO)
  #define BUTTON_A 4
  #define BUTTON_B 3
  #define BUTTON_C 8
  #define LED 13
#else 
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
  #define LED      13
#endif

#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// The service information
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

// globals 

// max9814 (mic board used for noise measurements)
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

// humidity, temperature, fahrenheit
float h,t,f;

// audio mic noise
double noise;

// light levels
uint32_t lum;
uint16_t ir, full, lux;

// Polar Heart Rate Monitor
Timer minute;
const int HR_RX = 10;
byte hr_oldSample, hr_sample;
int beatcount = 0;
int hrm_bpm = 0;

// average sensor data 
#define AVG_COUNT 2         // number of passes
int average_counter = 0;    // current location
float temp_avg,hum_avg;
uint16_t full_avg, lux_avg;
double noise_avg;
int heart_rate_avg;

double sensor_max_loops = 250000;
double sensor_loop = 0;

void setup() {  
  
  Serial.begin(9600);           // serial baud rate

  DEBUG_PRINTLN("setup begin");
  
  dht.begin();                  // temperature and humidity
  oled_setup();                 // OLED hardware display module for feather  
  tsl2561_light_sensor_setup(); // light level sensor
  heart_rate_monitor_setup();   // HRM Polar setup code 
  ble_communication_setup();          // bluetooth UART datalogging

  DEBUG_PRINTLN("finished setup");

}


void loop() {

  DEBUG_PRINTLN("loop begin");
  
  heart_rate_monitor_loop();   // heart rate monitor (any commercial HRM)    
  sensor_loop++;  

  if ( sensor_loop == sensor_max_loops ) {

  sensor_loop = 0;
  
  dht11_temp_humidity_loop();  // temperature and humdity DHT-11 sensor read and serial output  
  button_scan_loop();          // checking A, B and C buttons for a press on the OLED featherwing 
  max9814_noise_loop();        // audio mic noise levels   
  tsl2561_light_loop();        // visible light and lux  
  
  accumulate_sensors_loop();   // add up the sensor samples
  
  if ( average_counter == AVG_COUNT ) {
    
    average_sensors_loop();       // divide accumuluated sensor data by AVG_COUNT
    display_oled_loop();          // show sensor data on OLED featherwing
    ble_datalogging_loop();       // send to BlueFruit iOS app
    reset_average_samples_loop(); // reset values
        
  }
  
  }

  DEBUG_PRINTLN("loop finished");
} 


void ble_communication_setup() {
    /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));  
    if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
}


void oled_setup(){
    Serial.println("OLED FeatherWing test");
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  Serial.println("OLED begun");
 
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  
  Serial.println("IO test");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
}


void tsl2561_light_sensor_setup(){
    // light sensor
    if (tsl.begin()) {
    Serial.println("Found light sensor");
  } else {
    Serial.println("No light sensor?");
    while (1);
  }
    
  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)

}


void heart_rate_monitor_setup(){
    
  // every 60 seconds
  int tickEvent = minute.every(60000, average_beats);

  pinMode (HR_RX, INPUT);  //Signal pin to input  
 
  Serial.println("Waiting for heart beat...");

  //Wait until a heart beat is detected  
  while (!digitalRead(HR_RX)) {}; 
  Serial.println ("Heart beat detected!");
 
}


void dht11_temp_humidity_loop(){
    // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
   h = dht.readHumidity();
  // Read temperature as Celsius (the default)
   t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
   f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

}


void button_scan_loop(){
      
    if (! digitalRead(BUTTON_A)) display.print("A");
    if (! digitalRead(BUTTON_B)) display.print("B");
    if (! digitalRead(BUTTON_C)) display.print("C");

}


void max9814_noise_loop(){
  
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(0);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   noise = (peakToPeak * 3.3) / 1024;  // convert to volts
 
}


void tsl2561_light_loop(){
  
  // light
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  uint16_t x = tsl.getLuminosity(TSL2561_VISIBLE);     
  lum = tsl.getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;
  lux = tsl.calculateLux(full, ir);
 
}


void heart_rate_monitor_loop(){


  minute.update();
  
  hr_sample = digitalRead(HR_RX);  //Store signal output 

  if (hr_sample && (hr_oldSample != hr_sample)) {
    beatcount++;
  }

  hr_oldSample = hr_sample;           //Store last signal received 


}

void average_beats() {
  hrm_bpm = beatcount;
  Serial.print("beat count: ");
  Serial.println(hrm_bpm); 
  beatcount = 0;
  
}

void accumulate_sensors_loop(){

  temp_avg += f;
  hum_avg += h;
  full_avg += full;
  lux_avg += lux;
  noise_avg += noise;
  average_counter++;

}


void average_sensors_loop(){
  
  temp_avg = temp_avg / AVG_COUNT;
  hum_avg =  hum_avg / AVG_COUNT;
  full_avg = full_avg / AVG_COUNT;
  lux_avg = lux_avg / AVG_COUNT;
  noise_avg = noise_avg / AVG_COUNT;
  noise_avg *= 100;
}


void reset_average_samples_loop() {

  temp_avg = 0;
  hum_avg = 0;
  full_avg = 0;
  lux_avg = 0;
  noise_avg = 0;
  average_counter = 0;
  
}


void display_oled_loop(){

    
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Temp ");
  display.print(temp_avg);
  display.print(" Hum ");
  display.print(hum_avg);
  display.println();
  display.print("Light ");
  display.print(full_avg);
  display.print(" Lux ");
  display.print(lux_avg);
  display.println();
  display.print("Noise ");
  display.print(noise_avg);
  display.println();
  display.print("Heart Rate: ");
  display.print(hrm_bpm);
  display.display();
  
}


void ble_datalogging_loop(){

  char str_temp_avg[3], str_hum_avg[3], str_noise_avg[3];
  char buffer[20];
  ble.setMode(BLUEFRUIT_MODE_DATA);

  dtostrf(temp_avg,2,0,str_temp_avg);
  dtostrf(hum_avg,2,0,str_hum_avg);
  dtostrf(noise_avg,2,0,str_noise_avg);  
  
  snprintf(buffer, 20, "%s,%s,%d,%d,%s,%d", str_temp_avg, str_hum_avg, full_avg, lux_avg, str_noise_avg, hrm_bpm);
  
  ble.print(buffer);

}



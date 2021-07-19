#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "WiFi.h"
#include "ThingSpeak.h"
#define CHANNEL_ID 1329024 //for acsess channal in thingspeak server
#define CHANNEL_API_KEY "D30OO1MP6SHTXJQ4"

WiFiClient client; //for wifi connection bitween server

int counter  = 0;

#define WIFI_NETWORK "Malinda 4G"
#define WIFI_PASSWOR "voiceguide"
#define WIFI_TIMEOUT_MS 20000

void IRAM_ATTR onTimer(); //timer interrupt rutean
void connectToWiFi();


Adafruit_ADS1115 ads; //because we use ads1115 module

hw_timer_t* timer = NULL;//variables for timer functions
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int16_t results1; //these variables store the adc read values
volatile int16_t results2;
volatile int16_t results3;
volatile int16_t results4;
volatile float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
volatile uint8_t ledstat = 0;
volatile uint8_t flag = 0;
uint8_t ch = 0; //to track the channel of the ads1115 module
int16_t adc = 0;

void connectToWiFi(){ //mannage wifi connection
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA); //station mode
  WiFi.begin(WIFI_NETWORK,WIFI_PASSWOR);

  unsigned long startAttemptTime = millis();

  while(WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print(".");
    delay(100);
  }

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Failed!");
    //Reboot
  }else{
    Serial.print("Connected!");
    Serial.println(WiFi.localIP());
  }
}

void goToDeepSleep(){ //put the microcontroller into sleep.
  Serial.println("Going to sleep...");
  esp_sleep_enable_timer_wakeup(1000000*6); //wake me up in 6 secondes
  esp_deep_sleep_start();
}

void setup()
{
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);//cell 1 mosfet
  pinMode(4,OUTPUT);//cell 2 mosfet
  pinMode(5,OUTPUT);//cell 3 mosfet

  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  
  Wire.begin();//serial communication
  Serial.begin(115200); //serial communication speed set

  connectToWiFi(); //connect to wifi router

  ThingSpeak.begin(client); //connect to thingspeak server

  timer = timerBegin(0,80,true); //timer0 MWDT clock period = 12.5ns x 80
  timerAttachInterrupt(timer,&onTimer,true); //interrupt at edge trigger
  timerAlarmWrite(timer,15000000,true); //1s timer interrupt call, autoreload true
  timerAlarmEnable(timer); //enable  timer

  Serial.println("Getting single ended reading from AIN0, AIN1, AIN2 and AIN3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ch = 0;
}

void loop()
{

  ch++;
  if(ch > 3){
    ch = 0;
  }

  ads.startComparator_SingleEnded(ch, 3900); //looking at 3 channels of the ads1115 to check if overvoltage is detected.
  adc = ads.getLastConversionResults(); //if adc have a value it's mean overvoltage has happeded

  if(adc){
    switch(ch){
      case 0:
        digitalWrite(3,HIGH);
      break;
      case 1:
        digitalWrite(4,HIGH);
      break;
      case 2:
        digitalWrite(5,HIGH);
      break;
    }
    adc = 0;
  }

  if (flag == 1) //don't run until flag ==1
  {
    results1 = ads.readADC_SingleEnded(0); //get the reading
    Serial.print("single ended: "); Serial.print(results1 * multiplier); Serial.println("mV");

    results2 = ads.readADC_SingleEnded(1);
    Serial.print("single ended: "); Serial.print(results2 * multiplier); Serial.println("mV");

    results3 = ads.readADC_SingleEnded(2);
    Serial.print("single ended: "); Serial.print(results3 * multiplier); Serial.println("mV");

    results4 = ads.readADC_SingleEnded(3);
    Serial.print("single ended: "); Serial.print(results4 * multiplier); Serial.println("mV");

    Serial.println("----------------------------------------------------------------------");

    ThingSpeak.setField(1,(results1*multiplier));
    ThingSpeak.setField(2,(results2*multiplier));
    ThingSpeak.setField(3,(results3*multiplier));
    ThingSpeak.setField(4,(results4*multiplier));

    ThingSpeak.writeFields(CHANNEL_ID,CHANNEL_API_KEY); //push the values by connecting to wifi

    flag = 0;

    goToDeepSleep();
  }
  
}

void IRAM_ATTR onTimer(){ //timer interrupt
  portENTER_CRITICAL_ISR(&timerMux);

  ledstat = 1-ledstat;
  digitalWrite(2,ledstat); //light up the led

  flag = 1;

  portEXIT_CRITICAL_ISR(&timerMux);
}
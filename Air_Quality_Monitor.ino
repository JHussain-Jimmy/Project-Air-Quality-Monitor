// Include Libraries

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
//#include "LED.h"
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// initialization of sensors for temp, humidity and gas

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C protocol used


// LEDs initialized

const int redLed = 6;
const int greenLed = 5;
const int buttonPin = 4;     
const int blueLed =  3; 

// mp3 shield Initialization

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX Port on board
DFRobotDFPlayerMini myDFPlayer;

// Dust Sensor initialization

int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 10000; // matlab 10 second
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

//Push button with LED
/*turn on a LED when the button is pressed and let it on 
until the button is pressed again
*/

int brightness = 255;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by  
int buttonState = 0;
int flag=0;


// Adding weights to sensor variables to measure Air Quality

float hum_weighting = 0.25; // so humidity effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score
float hum_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;
int   getgasreference_count = 0;


// Function for Temperature, Humidity and Gas Sensor

void Air_Temp_Hum ()

{
  // temperature and humidity sensor ki logic
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");
  
  if (bme.readTemperature() > 22)
   {myDFPlayer.play(3);// it is so hot open window
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    delay(10000);} // To wait for sound clip to finish playing
    
  else if (bme.readTemperature() < 14)
    {myDFPlayer.play(4);// i am freezing turn up the heater
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    delay(10000);}// To wait for sound clip to finish playing
 
  Serial.print("   Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("   Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println("%");

  Serial.print("   Gas = ");
  Serial.print(bme.readGas());
  Serial.println("R\n");
 
 
 //Calculate humidity contribution to IAQ index
 
  float current_humidity = bme.readHumidity();
  if (current_humidity >= 38 && current_humidity <= 42)
    hum_score = 0.25*100; // Humidity +/-5% around optimum 
  else
  { //sub-optimal
    if (current_humidity < 38) 
      hum_score = 0.25/hum_reference*current_humidity*100;
    else
    {
      hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
    }
  }
  
  //Calculate gas contribution to IAQ index
  
  float gas_lower_limit = 5000;   // Bad air quality limit
  float gas_upper_limit = 50000;  // Good air quality limit 
  if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit; 
  if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
  gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;
  
  //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  float air_quality_score = hum_score + gas_score;

  Serial.println("Air Quality = "+String(air_quality_score,1)+"% derived from 25% of Humidity reading and 75% of Gas reading - 100% is good quality air");
  Serial.println("Humidity element was : "+String(hum_score/100)+" of 0.25");
  Serial.println("     Gas element was : "+String(gas_score/100)+" of 0.75");
  if (bme.readGas() < 120000) Serial.println("***** Poor air quality *****");
  Serial.println();
  if ((getgasreference_count++)%10==0) GetGasReference(); 
  Serial.println(CalculateIAQ(air_quality_score));
  Serial.println("------------------------------------------------");
  delay(2000);
  }

void GetGasReference(){
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  
  Serial.println("Getting a new gas reference value");
  int readings = 10;
  for (int i = 1; i <= readings; i++){ // read gas for 10 x 0.150mS = 1.5secs
    gas_reference += bme.readGas();
  }
  gas_reference = gas_reference / readings;
}

String CalculateIAQ(float score){
 
  String IAQ_text = "Air quality is ";
  score = (100-score)*5;
  if      (score >= 301)                  IAQ_text += "Hazardous";
  else if (score >= 201 && score <= 300 ) IAQ_text += "Very Unhealthy";
  else if (score >= 176 && score <= 200 ) IAQ_text += "Unhealthy";
  else if (score >= 151 && score <= 175 ) IAQ_text += "Unhealthy for Sensitive Groups";
  else if (score >=  51 && score <= 150 ) IAQ_text += "Moderate";
  else if (score >=  00 && score <=  50 ) IAQ_text += "Good";
  return IAQ_text;
}

void DustSensor ()
{
  lowpulseoccupancy = lowpulseoccupancy+duration;
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // integer percentage 0 to 100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // specification obtained from data sheet
    
    Serial.print("Concentration = ");
    Serial.print(concentration);
    Serial.println(" pcs/0.01cf");
    Serial.println("\n");
    
    //using dust sensor for air quality and smoke etc
    if (concentration <= 1.0) {
     Serial.println("It's a smokeless and dustless environment"); 
     myDFPlayer.play(1); //intro clip played
     LedDelay(greenLed, 10);
  }
    else if (concentration > 1.0 && concentration < 20000) {
     Serial.println("It's probably only you blowing air to the sensor :)"); 
     
    myDFPlayer.play(5); // too dusty here (clip played)
    LedDelay(redLed, 10);
    }
    
    else if (concentration > 20000 && concentration < 315000) {
     Serial.println("Smokes from matches detected!"); 

     myDFPlayer.play(6); // smoke from matches (clip played)
     LedDelay(redLed, 10);
     
    }
      else  { //(concentration > 315000)
     Serial.println("Smokes from cigarettes detected! Or It might be a huge fire! Beware!"); 
     LedDelay(redLed, 10); // clip played
  }
    
    
    lowpulseoccupancy = 0; // reset the variable for next iteration
  
  }


void LedDelay (int led, int del) // giving led pin number and delay wanted; giving it in sec
{
 
   starttime = millis(); // global var. getting current value of millis
  do 
  {
    buttonState = digitalRead(buttonPin); // read button state, most imp. thing
    delay(100); // continue reading for a 100 ms
    
    if (buttonState == LOW)
    break; // each iteration, checking for button state, if it is LOW meaning pressed, then break the loop here and go back to main 
    
    else
    {
    if (led == greenLed)
    {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    digitalWrite(blueLed, LOW);
    delay (10);
    }
    else if (led == blueLed)
    {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(blueLed, HIGH);
    delay (1000); // to make it blink, delay after on
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(blueLed, LOW);
    delay (1000); // delay after off
    }
    else if (led == redLed)
    {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    digitalWrite(blueLed, LOW);
    delay (10);
    }
    }
    }while (millis() - starttime < 1000*del); // It is in ms, 1000 is 1 sec
  }


// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
  //mp3 shield
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  //LEDs
  pinMode(greenLed,OUTPUT);
  pinMode(redLed,OUTPUT);
  pinMode(blueLed, OUTPUT); 
  
  //push button with led    
  pinMode(buttonPin, INPUT_PULLUP);

 //grove dust sensor
    pinMode(8,INPUT);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(12000); //Set serial communictaion time out 10s

  //----Set volume----
  myDFPlayer.volume(30);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  //myDFPlayer.volumeDown(); //Volume Down

  //----Set different EQ----
  //myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
//  myDFPlayer.EQ(DFPLAYER_EQ_POP);
//  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
//  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
//  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
 myDFPlayer.EQ(DFPLAYER_EQ_BASS);

  //----Set device we use SD as default----
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);

 
  // humidity, temp sensor's setup
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_2X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_2X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  GetGasReference();

  Air_Temp_Hum ();
  delay(2000);
      digitalWrite(blueLed, HIGH);  
      digitalWrite(greenLed, HIGH); 
      digitalWrite(redLed, HIGH);
  delay(5000);
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
  
  if(starting == 0) // first time code is running, check button state anyway
 {myDFPlayer.play(12); // intro of AQI
      digitalWrite(blueLed, HIGH);  
      digitalWrite(greenLed, LOW); 
      digitalWrite(redLed, LOW);
 delay(10000);
  buttonState = digitalRead(buttonPin); 
  starting = 2;} // never enter this loop again

// Then Read button state (pressed or not pressed?)in LedDelay function
//If button pressed...

  
  while (buttonState == LOW) 
  { 
    
    if ( flag == 0) // button was unpressed before
    {
     flag=1; //change flag variable to know it has been pressed
     myDFPlayer.pause(); // pause whatever crap you were saying
     myDFPlayer.play(speak4blue); //actual real-time state of sensors
     LedDelay (blueLed, 10);
  
      
    }
  
    else if ( flag == 1) // meaning button has been pressed and is probably already saying something
    { 
      flag=0; //change flag variable again
      myDFPlayer.pause();
      digitalWrite(blueLed, LOW); 
      digitalWrite(greenLed, LOW); 
      digitalWrite(redLed, LOW);  
     
    }  
    break;
  }
  
  
  while (buttonState != LOW) // button not pressed, continue reading rest of sensors
  {
    
  //dust sensor ki logic
  duration = pulseIn(pin, LOW);
  
  DustSensor();
   if (buttonState == LOW) // on way back from reading sensors, read button state again, pressed?
   break; // forget sensors, go out and outside condition will glow blue LEDs
 
  Air_Temp_Hum ();
  if (buttonState == LOW)
   break;
}


}// main loop function

/**********************************************************************
 * Modified Color View from Adafruit example
 * to run on ATMega32 chip with a push button to control the sensor led. 
 * Sends serial messages that are compatible with WPF Color Sensor app
 * Uses PB0 (pin 1) for for pushbutton input
 * Uses PB1 (pin 2) for led control output
 * SDA and SCL pins 22 and 23 connect to sensor
 * RX and TX pins 14 and 15 connect to USB to TTL serial cable TX and RX
 *********************************************************************/
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// pin to listen for button press
#define buttonPin PB0

// pin to control the sensor led
// when sensor led is tied to ground, the light will be off
#define sensorLedPin PB1

// a variable to track button presses
boolean buttonWasPressed = false;

// create an instance of the TCS34725 library to handle getting data from the sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Sensor...");

  if (tcs.begin()) 
  {
    Serial.println("Sensor Found.");
  } 
  else 
  {
    Serial.println("ERROR: TCS34725 Sensor NOT Found ... check your connections");
    while (1); // stop here, there is problem.
  }
    
  // button pin as input in data direction register
  DDRB &= ~(1 << buttonPin);
  
  // sensor led pin as output in data direction register
  DDRB |= (1 << sensorLedPin);
}

void loop() 
{
  CheckButton();
  
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);
  
  delay(60);  // takes 60ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);
  
  // scale rgb values
  uint32_t sum = clear;
  float r, g, b;
  r = red;   r /= sum; r *= 256;
  g = green; g /= sum; g *= 256;
  b = blue;  b /= sum; b *= 256;
    
  Serial.println();

  // Print Scaled RGB values
  Serial.print("RGB: "); 
  Serial.print((int)r ); Serial.print(" "); 
  Serial.print((int)g);  Serial.print(" ");  
  Serial.print((int)b ); Serial.print(" ");  
  PrintHEX((int)r);  PrintHEX((int)g); PrintHEX((int)b);
  Serial.print(" "); 
  Serial.print((PORTB & (1 << sensorLedPin)) ? 1 : 0); 
  Serial.print(" ");
  Serial.print(clear); Serial.print(" ");
  Serial.print(red); Serial.print(" ");
  Serial.print(green); Serial.print(" ");
  Serial.print(blue); Serial.print(" ");
}

// Toggle the led on or off if the button was pressed.
void CheckButton()
{  
  if(PINB & (1 << buttonPin))
  {
    // if this is a change from the last check
    // the button is pressed now, but wasn't before
    if(!buttonWasPressed)
    {
      // update the button pressed state
      buttonWasPressed = true;
      
      // toggle the led pin because the button was pressed and it was not previously being pressed
      PORTB ^= (1 << sensorLedPin); 
    }
    else
    {
      // The button is pressed but no change has occured
      // maybe the button is still pressed from before so 
      // do nothing
    }
  }
  else
  {
    // Don't update the led if button was not pressed again
    // just note that the button is not currently pressed so the 
    // next button press will be a state change
    buttonWasPressed = false;
  }
}

// Prints a 2 digit Hex number for the value passed in to the serial port
// if the value exceeds the maximum hex value, FF is printed
// if the value is less than 15 and so does not produce a value in the higher order 
// place value, this will put a 0 there
void PrintHEX(int value)
{
  if(value > 255)
  {
    // Cap the value to the maximum - print FF
    Serial.print(255, HEX); 
  }
  else if(value < 16)
  {
    // print a leading 0 in the most significant digit to keep the two digit format
    Serial.print(0, HEX);
    Serial.print(value, HEX);
  }
  else
  {
    Serial.print(value, HEX);
  }
}


#include <M5Stack.h>
#include <WiFi.h> 

const char* ssid       = "Buffalo-A-AC8E";
const char* password   = "5aidas7biba8u";

//the setup routine runs once ehen M5Stack starts up
void setup(){

  // Initialize the M5Stack object
  M5.begin();

  //LCD display
  M5.Lcd.printf ("hello world");
  
}

// the loop routine runs over and over again forever

void loop(){
  M5.Lcd.printf ("hello world");
  
}

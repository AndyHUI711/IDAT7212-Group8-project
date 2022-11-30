#include "arduino_secrets.h"
// ArduinoJson - Version: 6.14.0
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

// EspSoftwareSerial - Version: Latest
#include <SoftwareSerial.h>



/*
  Sketch generated by the Arduino IoT Cloud Thing "Neopixel"
  Arduino IoT Cloud Variables description
  The following variables are automatically generated and updated when changes are made to the Thing

  String plant;
  CloudColoredLight neopixel;
  CloudIlluminance light;
  int temperture;
  int waterLevel;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/
#include "thingProperties.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN            D7
#define NUMPIXELS      60
#define in_light       D5
#define mode           D6

#define lightinPin     A0


int mode_m = 1;
int switch_s = 0;


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);




  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);
  pixels.begin(); // This initializes the NeoPixel library.
  // Defined in thingProperties.h
  initProperties();
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  /*
    The following function allows you to obtain more information
    related to the state of network and IoT Cloud connection and errors
    the higher number the more granular information you’ll get.
    The default is 0 (only errors).
    Maximum is 4
  */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  //input
  pinMode(in_light, INPUT);
  pinMode(mode, INPUT);
  pinMode(lightinPin, INPUT);
  
  

}

String incomingByte ;
int val = 0;
bool swi;


void loop() {
  mode_m = digitalRead(mode);
  switch_s = digitalRead(in_light);


  
  
  char json = Serial.read();
  StaticJsonDocument<200> doc;

  
  const auto deser_err = deserializeJson(doc, Serial);
    if (deser_err) {
        Serial.print(F("Failed to deserialize, reason: \""));
        Serial.print(deser_err.c_str());
        Serial.println('"');
    } else  {
        Serial.print(F("Recevied valid json document with "));
        Serial.print(doc.size());
        Serial.println(F(" elements."));
        Serial.println(F("Pretty printed back at you:"));
        serializeJsonPretty(doc, Serial);
        
        switch_s = doc["OpenLight"];
        int temperature = doc["Temperture"];
        int lux = doc["Light"];
        int waterdata = doc["WaterLevel"];
        
        
  
        Serial.println();
    }
    
    if (switch_s == 1) {
    swi = true;
    //digitalRead(in_light)
    neopixel.setSwitch(swi);
    Serial.println(neopixel.getSwitch());
    onNeopixelChange();
    ArduinoCloud.update();
  }
  else {
    swi = false;
    //digitalRead(in_light)
    neopixel.setSwitch(swi);
    Serial.println(neopixel.getSwitch());
    onNeopixelChange();
    ArduinoCloud.update();
  }
  
  
  
  
  
  
  ArduinoCloud.update();


}

/*
  Since Neopixel is READ_WRITE variable, onNeopixelChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onNeopixelChange()  {
  // Add your code here to act upon Neopixel change
  uint8_t r, g, b;
  neopixel.getValue().getRGB(r, g, b);
  if (neopixel.getSwitch()) {
    swi = true;
    Serial.println("R:" + String(r) + " G:" + String(g) + " B:" + String(b)); //prints the current R, G, B values
    for (int i = 0; i < NUMPIXELS; i++) {
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(i, pixels.Color(r, g, b));
      pixels.show(); // This sends the updated pixel color to the hardware.
    }
  }
  else {

    Serial.println("Lamp Off");
    swi=false;

    //the following code simply turns everything off
    for (int i = 0; i < NUMPIXELS; i++) {
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show(); // This sends the updated pixel color to the hardware.
    }
  }
}

/*
  Since Light is READ_WRITE variable, onLightChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLightChange()  {
  // Add your code here to act upon Light change
}

/*
  Since Plant is READ_WRITE variable, onPlantChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onPlantChange()  {
  // Add your code here to act upon Plant change
}

/*
  Since Temperture is READ_WRITE variable, onTempertureChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTempertureChange()  {
  // Add your code here to act upon Temperture change
}

/*
  Since WaterLevel is READ_WRITE variable, onWaterLevelChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onWaterLevelChange()  {
  // Add your code here to act upon WaterLevel change
}



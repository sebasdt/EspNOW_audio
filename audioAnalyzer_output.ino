/*
  #include "sdkconfig.h"
  #include "driver/rtc_io.h"
  #include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/timers.h"
  #include "freertos/event_groups.h"*/
#include <Adafruit_NeoPixel.h>


int RFreqVal;
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

////
/* define event group and event bits */
EventGroupHandle_t eg;
////
QueueHandle_t queueReceved;
////
const int LED_COUNT = 50; //total number of leds in the strip
const int ledOFF = 5;
////
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
Adafruit_NeoPixel leds = Adafruit_NeoPixel( LED_COUNT, 13, NEO_RGB);
////
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  eg = xEventGroupCreate();
  //  leds.begin(); // Call this to start up the LED strip.
  //  clearLEDs( LED_COUNT );  // This function, defined below, de-energizes all LEDs...
  //  leds.show();  // ...but the LEDs don't actually update until you call this.
  ////
  queueReceved = xQueueCreate ( 1, sizeof(int));
  //////////////////////////////////////////////////////////////////////////////////////////////
  xTaskCreatePinnedToCore( fDo_LEDs, "fDo_ LEDs", 40000, NULL, 4, NULL, 0 ); //assigned to core
} // setup()
////
void loop() {
  vTaskDelete(NULL); // void loop
}
////
void fDo_LEDs( void *pvParameters )
{
  const int Brightness = 180;
  const int SEG = 6; // how many parts you want to separate the led strip into
  const int ledCount = LED_COUNT; //total number of leds in the strip
  int j;
  int FreqVal;
  leds.begin(); // Call this to start up the LED strip.
  clearLEDs( ledCount );  // This function, defined below, de-energizes all LEDs...
  leds.show();  // ...but the LEDs don't actually update until you call this.
  leds.setBrightness( Brightness ); //  1 = min brightness (off), 255 = max brightness.
  for (;;)
  {
    if (xQueueReceive( queueReceved, &FreqVal,  portMAX_DELAY) == pdTRUE)
    {
      j = 0; 
      Serial.println(FreqVal);

      //assign different values for different parts of the led strip
      for (j = 0; j < ledCount; j++)
      {
        if ( (0 <= j) && (j < (ledCount / SEG)) )
        {
          set( j, FreqVal, ledCount, SEG ); // set the color of led
        }
        else if ( ((ledCount / SEG) <= j) && (j < (ledCount / SEG * 2)) )
        {
          set( j, FreqVal, ledCount, SEG );
        }
        else if ( ((ledCount / SEG * 2) <= j) && (j < (ledCount / SEG * 3)) )
        {
          set( j, FreqVal, ledCount, SEG );
        }
        else if ( ((ledCount / SEG * 3) <= j) && (j < (ledCount / SEG * 4)) )
        {
          set( j, FreqVal, ledCount, SEG );
        }
        else if ( ((ledCount / SEG * 4) <= j) && (j < (ledCount / SEG * 5)) )
        {
          set( j, FreqVal, ledCount, SEG );
        }
        else
        {
          set( j, FreqVal, ledCount, SEG );
        }
      }
      leds.show();
    }

  }
  vTaskDelete( NULL );
} // void fDo_ LEDs( void *pvParameters )
////
//the following function set the led color based on its position and freq value
//
void set(byte position, int value, int ledCount, int segment)
{
  // segment 0, red
  if ( (0 <= position) && (position < ledCount / segment) ) // segment 0 (bottom to top), red
  {
    if ( value <= ledOFF )
    {
      leds.setPixelColor( position, 0, 0, 0 );
    }
    else
    {
      // increase light output of a low number
      // value += 10;
      // value = constrain( value, 0, 255 ); // keep raised value within limits
      leds.setPixelColor( position, leds.Color( value , 0, 0) );
    }
  }
  else if ( (ledCount / segment <= position) && (position < ledCount / segment * 2) ) // segment 1 yellow
  {
    if ( value <= ledOFF )
    {
      leds.setPixelColor(position, leds.Color(0, 0, 0));
    }
    else
    {
      leds.setPixelColor(position, leds.Color( value, value, 0)); // works better to make yellow
    }
  }
  else if ( (ledCount / segment * 2 <= position) && (position < ledCount / segment * 3) ) // segment 2 pink
  {
    if ( value <= ledOFF )
    {
      leds.setPixelColor(position, leds.Color(0, 0, 0));
    }
    else
    {
      leds.setPixelColor(position, leds.Color( value, 0, value * .91) ); // pink
    }
  }
  else if ( (ledCount / segment * 3 <= position) && (position < ledCount / segment * 4) ) // seg 3, green
  {
    if ( value <= ledOFF )
    {
      leds.setPixelColor(position, leds.Color( 0, 0, 0));
    }
    else //
    {
      leds.setPixelColor( position, leds.Color( 0, value, 0) ); //
    }
  }
  else if ( (ledCount / segment * 4 <= position) && (position < ledCount / segment * 5) ) // segment 4, leds.color( R, G, B ), blue
  {
    if ( value <= ledOFF )
    {
      leds.setPixelColor(position, leds.Color( 0, 0, 0));
    }
    else //
    {
      leds.setPixelColor(position, leds.Color( 0, 0, value) ); // blue
    }
  }
  else // segment 5
  {
    if ( value <= ledOFF )
    {
      leds.setPixelColor(position, leds.Color( 0, 0, 0)); // only helps a little bit in turning the leds off
    }
    else
    {
      leds.setPixelColor( position, leds.Color( value, value * .3, 0) ); // orange
    }
  }
} // void set(byte position, int value)
////
void clearLEDs( int ledCount)
{
  for (int i = 0; i < ledCount; i++)
  {
    leds.setPixelColor(i, 0);
  }
} // void clearLEDs()

// callback when data is recv from Master

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ///Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  //  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  // Serial.println("");
  int rFreqVal = *data;
  xQueueSend( queueReceved, &rFreqVal , portMAX_DELAY);
  //Serial.println(data);
}

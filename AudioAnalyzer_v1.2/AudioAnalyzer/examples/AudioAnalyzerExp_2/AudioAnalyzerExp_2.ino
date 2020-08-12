#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include <Adafruit_NeoPixel.h>
#include "AudioAnalyzer.h"
////
/* define event group and event bits */
EventGroupHandle_t eg;
#define evtDo_AudioReadFreq       ( 1 << 0 ) // 1
////
QueueHandle_t xQ_LED_Info;
////
const int LED_COUNT = 24; //total number of leds in the strip
////
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
Adafruit_NeoPixel leds = Adafruit_NeoPixel( LED_COUNT, 26, NEO_GRB + NEO_KHZ800 );
////
void setup()
{
  eg = xEventGroupCreate();
  //  Audio.Init(); // start the audio analyzer
  //  leds.begin(); // Call this to start up the LED strip.
  //  clearLEDs( LED_COUNT );  // This function, defined below, de-energizes all LEDs...
  //  leds.show();  // ...but the LEDs don't actually update until you call this.
  ////
  int FreqVal[7];
  xQ_LED_Info = xQueueCreate ( 1, sizeof(FreqVal) );
  //////////////////////////////////////////////////////////////////////////////////////////////
  xTaskCreatePinnedToCore( fDo_AudioReadFreq, "fDo_ AudioReadFreq", 40000, NULL, 4, NULL, 1 ); //assigned to core
  xTaskCreatePinnedToCore( fDo_LEDs, "fDo_ LEDs", 40000, NULL, 4, NULL, 0 ); //assigned to core
  xEventGroupSetBits( eg, evtDo_AudioReadFreq );
} // setup()
////
void loop() {} // void loop
////
void fDo_LEDs( void *pvParameters )
{
  const int Brightness = 180;
  const int SEG = 6; // how many parts you want to separate the led strip into
  const int ledCount = LED_COUNT; //total number of leds in the strip
  int iFreqVal[7];
  int j;
  leds.begin(); // Call this to start up the LED strip.
  clearLEDs( ledCount );  // This function, defined below, de-energizes all LEDs...
  leds.show();  // ...but the LEDs don't actually update until you call this.
  leds.setBrightness( Brightness ); //  1 = min brightness (off), 255 = max brightness.
  for (;;)
  {
    if (xQueueReceive( xQ_LED_Info, &iFreqVal,  portMAX_DELAY) == pdTRUE)
    {
      j = 0;
      //assign different values for different parts of the led strip
      for (j = 0; j < ledCount; j++)
      {
        if ( (0 <= j) && (j < (ledCount / SEG)) )
        {
          set( j, iFreqVal[0], ledCount, SEG ); // set the color of led
        }
        else if ( ((ledCount / SEG) <= j) && (j < (ledCount / SEG * 2)) )
        {
          set( j, iFreqVal[0], ledCount, SEG );
        }
        else if ( ((ledCount / SEG * 2) <= j) && (j < (ledCount / SEG * 3)) )
        {
          set( j, iFreqVal[0], ledCount, SEG );
        }
        else if ( ((ledCount / SEG * 3) <= j) && (j < (ledCount / SEG * 4)) )
        {
          set( j, iFreqVal[0], ledCount, SEG );
        }
        else if ( ((ledCount / SEG * 4) <= j) && (j < (ledCount / SEG * 5)) )
        {
          set( j, iFreqVal[0], ledCount, SEG );
        }
        else
        {
          set( j, iFreqVal[0], ledCount, SEG );
        }
      }
      leds.show();
    }
    xEventGroupSetBits( eg, evtDo_AudioReadFreq );
  }
  vTaskDelete( NULL );
} // void fDo_ LEDs( void *pvParameters )
////
void fDo_AudioReadFreq( void *pvParameters )
{
  int FreqVal[7];
  const int NOISE = 10; // noise that you want to chop off
  const int A_D_ConversionBits = 4096; // arduino use 1024, ESP32 use 4096
  Analyzer Audio = Analyzer( 5, 15, 36 );//Strobe pin ->15  RST pin ->2 Analog Pin ->36
  Audio.Init(); // start the audio analyzer
  int64_t EndTime = esp_timer_get_time();
  int64_t StartTime = esp_timer_get_time(); //gets time in uSeconds like Arduino Micros
  for (;;)
  {
    xEventGroupWaitBits (eg, evtDo_AudioReadFreq, pdTRUE, pdTRUE, portMAX_DELAY);
    EndTime = esp_timer_get_time() - StartTime;
    // log_i( "TimeSpentOnTasks: %d", EndTime );
    Audio.ReadFreq(FreqVal);
    for (int i = 0; i < 7; i++)
    {
      FreqVal[i] = constrain( FreqVal[i], NOISE, A_D_ConversionBits );
      FreqVal[i] = map( FreqVal[i], NOISE, A_D_ConversionBits, 0, 255 );
      // log_i( "Freq %d Value: %d", i, FreqVal[i]);//used for debugging and Freq choosing
    }
    xQueueSend( xQ_LED_Info, ( void * ) &FreqVal, 0 );
    StartTime = esp_timer_get_time();
  }
  vTaskDelete( NULL );
} // fDo_ AudioReadFreq( void *pvParameters )
////
//the following function set the led color based on its position and freq value
//
void set(byte position, int value, int ledCount, int segment)
{
  // segment 0, red
  if ( (0 <= position) && (position < ledCount / segment) ) // segment 0 (bottom to top), red
  {
    if ( value == 0 )
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
    if ( value == 0 )
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
    if ( value == 0 )
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
    if ( value == 0 )
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
    if ( value == 0 )
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
    if ( value == 0 )
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

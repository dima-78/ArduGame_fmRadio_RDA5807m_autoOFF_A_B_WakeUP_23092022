/*
   Used radio FM tuner RDA5807m in West-FM mode
   (87-108MHz)for ArduGame v1.2 (Arduboy/ArduboyFX)
   Radio module RDA5807m should be connected to a 3.3V ArduGame v1.2 (Arduboy/ArduboyFX)
   I2C interface.
   Special thanks Mr.Blinky for power-down librarie
   Copyright: (C) 2022
   Made by PetrovD (Dmtry78)
   12.09.2022
   23.09.2022 add: logo intro, clearing text from right to left, change delays.  
*/
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <wiring.c>
#include <util/atomic.h>

#include <Arduboy2.h>
//#include <Tinyfont.h>
#include <ArduboyTones.h>
#include <SPI.h>
//Due to a bug in Arduino, this needs to be included here too/first
#include <Wire.h>
#include <radio.h>
//Add the RDA5807M Library to the sketch.
#include <RDA5807M.h>
//#include <si4703.h>
//#include <si4705.h>
//#include <tea5767.h>
#include <RDSParser.h>

#define FIX_VOLUME   1
#define FIX_BAND     RADIO_BAND_FM   ///< The band that will be tuned by this sketch is FM.
#define FIX_STATION  9920//9760//9030 ///< The station that will be tuned by this sketch is 90.60 MHz.

/*************************************************************************************************

   INTRO SCREEN

 **************************************************************************************************/

const uint8_t PROGMEM logo[] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0xfc, 0xff, 0x3f, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x80, 0xf0, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xfc, 0xfe, 0xff, 0xfe, 0x3e, 0x3e,
  0x3e, 0x3e, 0x3e, 0x1e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0c, 0xf0, 0xfc, 0xfe, 0xff, 0xfe,
  0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xe0, 0xf8, 0xfc, 0xfe, 0xff, 0xfe, 0xfc, 0x3c,
  0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xe0, 0xf8, 0xff, 0x7f, 0x1f, 0x03, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7b, 0x00, 0xf8,
  0xfc, 0xf8, 0xe0, 0xe0, 0xf0, 0xf0, 0x70, 0x70, 0x60, 0x00, 0x80, 0xc0, 0xe0, 0xe0, 0xf0, 0xf0,
  0x70, 0xf0, 0xf8, 0xff, 0xff, 0x0f, 0x01, 0x00, 0xe0, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe0, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x80, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf8,
  0xf8, 0xf8, 0xf8, 0x38, 0x38, 0x38, 0x30, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0x3f, 0x0f,
  0x7f, 0xff, 0xf8, 0xe0, 0xf0, 0xfc, 0xff, 0x3f, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0xfc,
  0xff, 0xff, 0x3f, 0x7b, 0x70, 0x70, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0xfc, 0xff,
  0x7f, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0xf8, 0xfc, 0xff, 0x1f, 0x0f, 0x03, 0x01, 0x00, 0x00,
  0x80, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0xfe, 0xff, 0x3f, 0x07, 0x00, 0x00, 0x00, 0xfc, 0xff,
  0xff, 0x0f, 0x01, 0x00, 0x00, 0x80, 0xf8, 0xff, 0xff, 0xff, 0x1f, 0x03, 0x01, 0x01, 0x01, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xf0, 0xfc, 0xff, 0xff, 0x3f, 0x0f, 0x03, 0x00, 0x00,
  0x00, 0x03, 0x07, 0x07, 0x07, 0x01, 0xc0, 0xfc, 0xff, 0xff, 0xff, 0x3f, 0x07, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x1f, 0x1f, 0x07, 0x03,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x1f, 0x1f, 0x1f, 0x0f, 0x00, 0x03, 0x07, 0x07,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f,
  0x07, 0x03, 0x03, 0x01, 0x00, 0x00, 0x03, 0x07, 0x0f, 0x0e, 0x0e, 0x0e, 0x0e, 0x0f, 0x0f, 0x0f,
  0x03, 0x00, 0x00, 0x00, 0x02, 0x07, 0x07, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x0f, 0x0f, 0x0f, 0x1f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0f, 0x0f, 0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*************************************************************************************************

   POWER DOWN

 **************************************************************************************************/
void autoPowerDownReset();

void autoPowerDown(uint8_t timeout = 8); //~32,8 sec

extern uint16_t APD_time;

#ifdef AB_DEVKIT
#error "Arduboy DevKit is not supported"
#endif

/*******************************************************************************
   INT6_enable
   Macro to Enable INT6 interrupt on low level. (attachInterrupt)
 ******************************************************************************/

#define INT6_enable() \
  EICRB &= ~(ISC61 || ISC60); \
  EIMSK |= _BV(INT6);

/*******************************************************************************
   INT6_disable
   Macro to Disable INT6 interrupt (detachInterrupt)
 ******************************************************************************/

#define INT6_disable() \
  EIMSK &= ~_BV(INT6);

/*******************************************************************************
   PCINT0_enable
   Macro to enable Pin change interrupt (attachInterrupt)
 ******************************************************************************/

#define PCINT0_enable()  \
  PCICR  |= _BV(PCIE0); \
  PCMSK0 |= _BV(PCINT4);

/*******************************************************************************
   PCINT0_disable
   Macro to Disable Pin Change interrupt (detachInterrupt)
 ******************************************************************************/

#define PCINT0_disable() \
  PCICR &= ~_BV(PCIE0);

/*******************************************************************************
  activatePowerDown()
  Macro to activate power down
*******************************************************************************/

#define activatePowerDown()                                       \
  cli();                                                          \
  SMCR = SLEEP_MODE_PWR_DOWN | _BV(SE); /* sleep mode + enable */ \
  sei();                                                          \
  sleep_cpu();                                                    \
  SMCR = 0;                             /* sleep disable*/        \
  sei();

/*******************************************************************************
  all_LEDs_off()
  Macro to turn all LEDs off including PWM ones
*******************************************************************************/

#ifndef AB_ALTERNATE_WIRING
#define all_LEDs_off()                                  \
  TCCR1A = 0; /* disable Arduboy RGB LED PWMs */        \
  Arduboy2::digitalWriteRGB(RGB_OFF, RGB_OFF, RGB_OFF); \
  TXLED0; RXLED0; /* Rx Tx LEDs off */
#else
#define all_LEDs_off()                                      \
  TCCR1A = 0; /* disable Pro Micro BLUE and RED LED PWMs */ \
  TCCR0A = 0; /* disable Pro Micro GREEN LED PWM */         \
  Arduboy2::digitalWriteRGB(RGB_OFF, RGB_OFF, RGB_OFF);     \
  TXLED0; RXLED0; /* Rx Tx LEDs off */
#endif

uint16_t APD_time;
/*******************************************************************************
  INT6 external interrupt service routine
  Executed when the A-BUTTON is pressed
*******************************************************************************/

EMPTY_INTERRUPT (INT6_vect)

/*******************************************************************************
  PCINT0 pinchange interrupt service routine
  Executed when the B-BUTTON is pressed
*******************************************************************************/

EMPTY_INTERRUPT (PCINT0_vect)

/*******************************************************************************
  autoPowerDownReset

  resets auto power down millis to current millis
*******************************************************************************/
//unsigned char buttonsIdleTime(void);

void autoPowerDownReset()
{
  uint16_t time;
  asm volatile(
    "    in    __tmp_reg__, __SREG__    \n\t" //uint8_t oldSREG = SREG;
    "    cli                            \n\t" //cli();
    "    lds   %A[time], %[millis1]     \n\t" //time = *(uint16_t *)(&millis + 1);
    "    lds   %B[time], %[millis2]     \n\t"
    "    out   __SREG__, __tmp_reg__    \n\t" //SREG = oldSREG;
    "    sts   %[APD_time0], %A[time]   \n\t" //APD_time = time
    "    sts   %[APD_time1], %B[time]   \n\t"
    : [time]      "=&r" (time)
    : [millis1]   ""    ((uint8_t*)(&timer0_millis) + 1),
    [millis2]   ""    ((uint8_t*)(&timer0_millis) + 2),
    [APD_time0] ""  ((uint8_t*)(&APD_time) + 0),
    [APD_time1] ""  ((uint8_t*)(&APD_time) + 1)
  );
}

/*******************************************************************************
  autoPowerDown
*******************************************************************************/

void autoPowerDown(uint8_t timeout)
{
  //#ifndef ARDUBOY_CORE
  uint16_t time;
  asm volatile(
    "    in    __tmp_reg__, __SREG__    \n\t" //uint8_t oldSREG = SREG;
    "    cli                            \n\t" //cli()
    "    lds   %A[time], %[millis1]     \n\t" //time = (uint16_t)(millis >>8)
    "    lds   %B[time], %[millis2]     \n\t"
    "    out   __SREG__, __tmp_reg__    \n\t" //SREG = oldSREG
    "    sub   %A[time], %A[APD_time]   \n\t" //time -= APD_time
    "    sbc   %B[time], %B[APD_time]   \n\t"
    "1:                                 \n\t"
    : [time]     "=&r" (time)
    : [millis1]  ""    ((uint8_t*)(&timer0_millis)+1),
    [millis2]  ""    ((uint8_t*)(&timer0_millis)+2),
    [APD_time] "r"   (APD_time)
  );

  if (time >= timeout * 16)
    //power down timeout has been reached, so power down
    /*
      #else
      if (buttonsIdleTime() >= timeout)
      #endif
      {
      #ifndef ARDUBOY_NO_USB
        UDIEN = 0;            //disable USB interrupts left enabled after upload
        USBCON = _BV(FRZCLK); //disable VBUS transition interrupt, freeze USB clock for power savings
        UDCON  = 1 << DETACH; //disconnect from USB bus
      #endif
        all_LEDs_off();
        Arduboy2Core::displayOff();
        INT6_enable();               //enable A-button interrupt so Arduboy can wake up by pressing it.
        PCINT0_enable();             //enable B-button interrupt so Arduboy can wake up by pressing it.
        activatePowerDown();
        INT6_disable();              //disable A-button interrupt. No unneccesary interrupts wanted.
        PCINT0_disable();            //disable B-button interrupt. No unneccesary interrupts wanted.
      #ifndef ARDUBOY_NO_USB
        init();                      //restore USB support
        USBDevice.attach();
      #endif
        Arduboy2Core::displayOn();
      #ifndef ARDUBOY_CORE
        autoPowerDownReset();
      #endif
      }
      #ifndef ARDUBOY_CORE
      if (Arduboy2Core::buttonsState()) autoPowerDownReset(); //reset power down time on any button press
      #endif
      }
    */
  {
    if (UHWCON & _BV(UVREGE)) // Test if USB is enabled
    {
      UDIEN = 0;            //disable USB interrupts left enabled after upload
      USBCON = _BV(FRZCLK); //disable VBUS transition interrupt, freeze USB clock for power savings
      UDCON  = 1 << DETACH; //disconnect from USB bus
    }
    all_LEDs_off();
    Arduboy2Core::displayOff();
    INT6_enable();               //enable A-button interrupt so Arduboy can wake up by pressing it.
    PCINT0_enable();             //enable B-button interrupt so Arduboy can wake up by pressing it.
    activatePowerDown();
    INT6_disable();              //disable A-button interrupt. No unneccesary interrupts wanted.
    PCINT0_disable();            //disable B-button interrupt. No unneccesary interrupts wanted.
    if (UHWCON & _BV(UVREGE)) // Test if USB is enabled
    {
      init();                      //restore USB support
      USBDevice.attach();
    }
    Arduboy2Core::displayOn();
    autoPowerDownReset();
  }
  if (Arduboy2Core::buttonsState()) autoPowerDownReset(); //reset power down time on any button press
}

const unsigned char PROGMEM Radio [] = {0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0x40, 0x00, 0x00, 0x00, 0x02, 0x80, 0x00, 0x00, 0xC0, 0x01, 0x80, 0x00, 0x00, 0xC0, 0x01, 0x8E, 0x88, 0x00, 0xC0, 0x01, 0x88, 0xD8, 0x00,
                                        0xC0, 0x01, 0x88, 0xA8, 0x00, 0xC0, 0x01, 0x8C, 0xA8, 0x00, 0xC0, 0x01, 0x88, 0xA8, 0x00, 0xC0, 0x01, 0x80, 0x00, 0x00, 0xC0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x81, 0x04, 0x10, 0x41, 0x05, 0x81, 0x04, 0x10,
                                        0x41, 0x05, 0x95, 0x55, 0x55, 0x55, 0x55, 0x95, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00,
                                        0xF8, 0x01, 0x80, 0x00, 0x03, 0xFE, 0x01, 0x80, 0x00, 0x07, 0x03, 0x01, 0x80, 0x00, 0x0C, 0x01, 0x81, 0x80, 0x60, 0x18, 0x00, 0xC1, 0x81, 0xF8, 0x18, 0x00, 0x41, 0x83, 0x9C, 0x10, 0x00, 0x61, 0x83, 0x0C, 0x30,
                                        0x00, 0x61, 0x83, 0x0C, 0x30, 0x00, 0x61, 0x83, 0x0C, 0x30, 0x00, 0x61, 0x83, 0x9C, 0x10, 0x00, 0x61, 0x81, 0xF8, 0x18, 0x00, 0x41, 0x80, 0x60, 0x18, 0x00, 0xC1, 0x80, 0x00, 0x0C, 0x01, 0x81, 0x80, 0x00, 0x07,
                                        0x03, 0x01, 0x80, 0x00, 0x03, 0xFE, 0x01, 0x80, 0x00, 0x00, 0xF8, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x00, 0x02, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC
                                       };

const unsigned char PROGMEM Volume [] = {0x7E, 0x42, 0x81, 0x81, 0x81, 0x81, 0xE7, 0xE7, 0x66, 0x66};
const unsigned char PROGMEM Bass [] = {0x7F, 0xFE, 0x80, 0x01, 0x9B, 0x8D, 0xA2, 0x51, 0xA3, 0x91, 0x9A, 0x4D, 0x8A, 0x45, 0xB3, 0x99, 0x80, 0x01, 0x7F, 0xFE};
const unsigned char PROGMEM FM [] = {0x7F, 0xFE, 0x80, 0x01, 0x8C, 0x51, 0x90, 0xA9, 0x90, 0xA9, 0x98, 0xA9, 0x90, 0xA9, 0x90, 0x89, 0x80, 0x01, 0x7F, 0xFE};
const unsigned char PROGMEM Stereo [] = {0x7F, 0xFE, 0x80, 0x01, 0x9B, 0xB9, 0xA1, 0x25, 0xA1, 0x39, 0x99, 0x31, 0x89, 0x29, 0xB1, 0x25, 0x80, 0x01, 0x7F, 0xFE};
const unsigned char PROGMEM SM [] = {0x7F, 0xFE, 0x80, 0x01, 0x8C, 0x51, 0x90, 0xA9, 0x90, 0xA9, 0x8C, 0xA9, 0x84, 0xA9, 0x98, 0x89, 0x80, 0x01, 0x7F, 0xFE};

const unsigned char PROGMEM Bat1[] = {0x7F, 0xFC, 0x80, 0x02, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x02, 0x7F, 0xFC};
const unsigned char PROGMEM Bat2[] = {0x7F, 0xFC, 0x80, 0x02, 0xA0, 0x01, 0xA0, 0x01, 0xA0, 0x01, 0xA0, 0x01, 0xA0, 0x01, 0xA0, 0x01, 0x80, 0x02, 0x7F, 0xFC};
const unsigned char PROGMEM Bat3[] = {0x7F, 0xFC, 0x80, 0x02, 0xA8, 0x01, 0xA8, 0x01, 0xA8, 0x01, 0xA8, 0x01, 0xA8, 0x01, 0xA8, 0x01, 0x80, 0x02, 0x7F, 0xFC};
const unsigned char PROGMEM Bat4[] = {0x7F, 0xFC, 0x80, 0x02, 0xAA, 0x01, 0xAA, 0x01, 0xAA, 0x01, 0xAA, 0x01, 0xAA, 0x01, 0xAA, 0x01, 0x80, 0x02, 0x7F, 0xFC};
const unsigned char PROGMEM Bat5[] = {0x7F, 0xFC, 0x80, 0x02, 0xAA, 0x81, 0xAA, 0x81, 0xAA, 0x81, 0xAA, 0x81, 0xAA, 0x81, 0xAA, 0x81, 0x80, 0x02, 0x7F, 0xFC};
const unsigned char PROGMEM Bat6[] = {0x7F, 0xFC, 0x80, 0x02, 0xAA, 0xA1, 0xAA, 0xA1, 0xAA, 0xA1, 0xAA, 0xA1, 0xAA, 0xA1, 0xAA, 0xA1, 0x80, 0x02, 0x7F, 0xFC};
const unsigned char PROGMEM Bat7[] = {0x7F, 0xFC, 0x80, 0x02, 0xAA, 0xA9, 0xAA, 0xAD, 0xAA, 0xAD, 0xAA, 0xAD, 0xAA, 0xAD, 0xAA, 0xA9, 0x80, 0x02, 0x7F, 0xFC};

const unsigned char PROGMEM Vol1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00};
const unsigned char PROGMEM Vol2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x6C, 0x00, 0x6C, 0x00};
const unsigned char PROGMEM Vol3[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01, 0x80, 0x0D, 0x80, 0x0D, 0x80, 0x6D, 0x80, 0x6D, 0x80};
const unsigned char PROGMEM Vol4[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x01, 0xB0, 0x01, 0xB0, 0x0D, 0xB0, 0x0D, 0xB0, 0x6D, 0xB0, 0x6D, 0xB0};
const unsigned char PROGMEM Vol5[] = {0x00, 0x06, 0x00, 0x06, 0x00, 0x36, 0x00, 0x36, 0x01, 0xB6, 0x01, 0xB6, 0x0D, 0xB6, 0x0D, 0xB6, 0x6D, 0xB6, 0x6D, 0xB6};

byte v, k, m, rs, flag1 = 1, flag2 = 0, flag3 = 0;
unsigned int s, s2;
boolean b, ms, sm, flag = 1, MODE = 1;
long TimeOut;

//TODO:
RADIO_FREQ preset[] = {
  8770,
  8810, //
  8820,
  8850, //
  8890, //
  8930, //
  8980,
  9060,
  9180,
  9220,
  9350,
  9440, //
  9510, //
  9530,
  9560, //
  9680,
  9880,
  10020, //
  10090, //
  10110, //
  10120,
  10030,
  10260,
  10380,
  10400,
  10500 //
};

// get a RDS parser
RDSParser rds;

// State definition for this radio implementation.
enum RADIO_STATE {
  STATE_PARSECOMMAND, ///< waiting for a new command character.

  STATE_PARSEINT,     ///< waiting for digits for the parameter.
  STATE_EXEC          ///< executing the command.
};

RADIO_STATE state; ///< The state variable is used for parsing input characters.

Arduboy2 arduboy;

//const int wakeUpPin = digitalRead(7);

//Tinyfont tinyfont = Tinyfont(arduboy.sBuffer, Arduboy2::width(), Arduboy2::height());
ArduboyTones sound(arduboy.audio.enabled);
//BeepPin1 beep;

constexpr uint8_t frameRate = 40; // Frame rate in frames per second

#define BUTTON_DELAY 200

byte circlePos = 7;

//Create an instance of the RDA5807M named radio
//RADIO radio;       ///< Create an instance of a non functional radio.
RDA5807M radio;    ///< Create an instance of a RDA5807 chip radio

//char command;
word status, frequency;
boolean mute = false;   //If the radio has been mute

void DisplayFrequency(RADIO_FREQ f)
{
  char s[26];
  radio.formatFrequency(s, sizeof(s));
  arduboy.setCursor(37, 35); arduboy.print(s);
} // DisplayFrequency()

// This function will be called by the RDS module when a new ServiceName is available.
// Update the LCD to display the ServiceName in row 1 chars 0 to 7.
void DisplayServiceName(char *name)
{
  size_t len = strlen(name);
  //arduboy.setCursor(40, 1);
  arduboy.setCursor(43, 22);
  arduboy.print(name);
  while (len < 8) {
    arduboy.print(' ');
    len++;
  } // while
} // DisplayServiceName()

// This function will be called by the RDS module when a rds time message was received.
// Update the LCD to display the time in right upper corner.

void DisplayTime(uint8_t hour, uint8_t minute) {
  arduboy.setCursor(50, 12);
  if (hour < 10) arduboy.print('0');
  arduboy.print(hour);
  arduboy.print(':');
  if (minute < 10) arduboy.print('0');
  arduboy.println(minute);
} // DisplayTime()


void RDS_process(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4) {
  rds.processData(block1, block2, block3, block4);
}

void moveCircle_L_R() {
  while (true) {
    arduboy.fillCircle(circlePos, 54, 7, BLACK);
    circlePos += 8;
    if (circlePos > 120) 
    {
      circlePos = 7;
      break;
    }
    arduboy.fillCircle(circlePos, 54, 7, WHITE);
    arduboy.display();
    delay(50); //was delay(100);
  }
}

void moveCircle_R_L() 
{
  while (true) {
    arduboy.fillCircle(circlePos, 54, 7, BLACK);
    circlePos -= 8;
    if (circlePos > 120)   
    {
      circlePos = 120;
      break;
    }
    arduboy.fillCircle(circlePos, 54, 7, WHITE);
    arduboy.display();
    delay(50); //was delay(100);
  }
}

//===================================================
int Voltage;

void readVcc()
{
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = 1.080 * 1023 * 1000 / result; // VCC real calculate
  Voltage = int(result);
  Voltage = map(Voltage, 3200, 4200, 0, 100);
  Voltage = constrain(Voltage, 0, 100);
}
//===================================================
void viewVolts () { //TODO
  /*
    if (Voltage > 85) arduboy.drawBitmap (1, 0, 2, 10, Bat7);
      else if (Voltage > 70) arduboy.drawBitmap (1, 0, 2, 10, Bat6);
      else if (Voltage > 55) arduboy.drawBitmap (1, 0, 2, 10, Bat5);
      else if (Voltage > 40) arduboy.drawBitmap (1, 0, 2, 10, Bat4);
      else if (Voltage > 25) arduboy.drawBitmap (1, 0, 2, 10, Bat3);
      else if (Voltage > 10) arduboy.drawBitmap (1, 0, 2, 10, Bat2);
      else arduboy.drawBitmap (1, 0, 2, 10, Bat1);
  */
  //arduboy.clear();
  if (Voltage > 85) arduboy.drawBitmap (5, 2, Bat7, 20, 2, WHITE);
  else if (Voltage > 70) arduboy.drawBitmap (5, 2, Bat6, 20, 2, WHITE);
  else if (Voltage > 55) arduboy.drawBitmap (5, 2, Bat5, 20, 2, WHITE);
  else if (Voltage > 40) arduboy.drawBitmap (5, 2, Bat4, 20, 2, WHITE);
  else if (Voltage > 25) arduboy.drawBitmap (5, 2, Bat3, 20, 2, WHITE);
  else if (Voltage > 10) arduboy.drawBitmap (5, 2, Bat2, 20, 2, WHITE);
  else arduboy.drawBitmap (5, 2, Bat1, 20, 2, WHITE);
  arduboy.display();
}

//================================================
/*
  void _displayOff()
  {
  if (MODE == 1 && millis() - TimeOut > 2000) flag = 1;
  if (MODE == 1 && millis() - TimeOut > 8000)
  {
    arduboy.displayOn();
    MODE = 0;

    if (v != EEPROM[1]) EEPROM[1] = v;
    if (k != EEPROM[2]) EEPROM[2] = k;
    if (m != EEPROM[3]) EEPROM[3] = m;
    if (b != EEPROM[4]) EEPROM[4] = b;
    if (ms != EEPROM[5]) EEPROM[5] = ms;
    if (s2 != s) {
      s2 = s;   EEPROM[6] = int(s / 100);
      EEPROM[7] = s % 100;
    }
    if (sm != EEPROM[8]) EEPROM[8] = sm;
  }
  if (MODE == 0 && millis() - TimeOut < 500)
  {
    arduboy.displayOff();
    //readVcc();
    MODE = 1;
    flag = 1;
    m = 7;
  }
  }
*/
void GoToSleep() {
  digitalWrite(GREEN_LED, HIGH); // turn off green LED
  digitalWrite(RED_LED, HIGH);// turn off red LED
  arduboy.displayOff();//Turn the Screen OFF
  arduboy.delayShort(500);// Buy a little time before power down
  // Enter power down state with ADC disabled and BOD module enabled.
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // Disable external pin interrupt on wake up pin.
  //detachInterrupt(4);
}

/*
  #define ARDBITMAP_SBUF arduboy.getBuffer()
  #include <ArdBitmap.h>
  ArdBitmap<WIDTH, HEIGHT> ardbitmap;
*/
//extern uint8_t logo[];

void setup() {
  //Serial.begin(57600);
  // Serial.println("Radio...");
  //delay(200);
  autoPowerDownReset();
  arduboy.begin();
  //arduboy.setRGBled(0,0,0);
  //arduboy.beginNoLogo();
  PRR0 &= ~_BV(PRTWI); //Power Riduction Register...Power Riduction TWI
  //pinMode(wakeUpPin, INPUT);
  //beep.begin();
  // here we set the framerate to 30, we do not need to run at default 60 and
  // it saves us battery life.
  arduboy.setFrameRate(20);

  arduboy.clear();
  arduboy.drawBitmap (0, 0, logo, 128, 64, WHITE);
  arduboy.display();
  delay(3000);
  arduboy.clear();
  /*
    arduboy.print(F("ArduFM tuner RDA5807\n"
                   "Dpad: Up-VolumeUP\n"
                   "Dpad: Down-VolumeDOWN\n"
                   "Dpad: Right-SeekingUP\n"
                   "Dpad: Left-SeekingDWN\n"
                   "   A: BassBoostON\n"
                   "   B: BassBoostOFF\n"
                   "  Press A to start"));
  */
  arduboy.print(F(" ArduFM tuner RDA5807\n"
                  "Dpad: Up-VolumeUP\n"
                  "Dpad: Down-VolumeDOWN\n"
                  "Dpad: Right-SeekingUP\n"
                  "Dpad: Left-SeekingDWN\n"
                  "   A: DisplayON\n"
                  "   B: DisplayOFF\n"
                  "  Press A to start"));
  arduboy.display();
  while (!arduboy.pressed(A_BUTTON)) {
    arduboy.idle();
  }
  arduboy.clear();
  arduboy.setCursor(28, 54);
  arduboy.print("FM_Radio Tuner");
  //for (int i = 0; i <5000; i++); //clear cicle
  moveCircle_L_R();
  delay(100);
  radio.init();

  // Enable information to the Serial port
  // radio.debugEnable();

  radio.setBandFrequency(FIX_BAND, FIX_STATION);
  radio.setMono(false);
  radio.setMute(false);
  radio.setBassBoost(false);
  radio.setVolume(FIX_VOLUME);
  state = STATE_PARSECOMMAND;
  // setup the information chain for RDS data.
  radio.attachReceiveRDS(RDS_process);
  rds.attachServicenNameCallback(DisplayServiceName);
  rds.attachTimeCallback(DisplayTime);
  //readVcc();
}

void loop() {
  //viewVolts();
  /*
    char s[12];
    radio.formatFrequency(s, sizeof(s));
    Serial.print("Station:");
    Serial.println(s);

    Serial.print("Radio:");
    radio.debugRadioInfo();

    Serial.print("Audio:");
    radio.debugAudioInfo();
  */
  // pause render until it's time for the next frame
  if (!(arduboy.nextFrame())) return;
  unsigned long now = millis();
  static unsigned long nextFreqTime = 0;
  static unsigned long nextRadioInfoTime = 0;
  // some internal static values for parsing the input
  static char command;
  static int16_t value;
  static RADIO_FREQ lastf = 0;
  RADIO_FREQ f = 0;

  arduboy.display();

  //while (true) {
  if (arduboy.pressed(UP_BUTTON)) {
    arduboy.setCursor(43, 50);
    arduboy.print("VolumeUP");
    sound.tone(1500);
    int v = radio.getVolume();
    if (v < 15) radio.setVolume(++v);
    delay(50);
    sound.noTone();
    moveCircle_L_R();
  }
  if (arduboy.pressed(DOWN_BUTTON)) {
    arduboy.setCursor(38, 50);
    arduboy.print("VolumeDOWN");
    sound.tone(1500);
    int v = radio.getVolume();
    if (v > 0) radio.setVolume(--v);
    delay(50);
    sound.noTone();
    moveCircle_R_L();
  }
  if (arduboy.pressed(RIGHT_BUTTON)) {
    radio.seekUp();
    sound.tone(2000);
    arduboy.setCursor(42, 50);
    delay(50); //was delay(100);
    sound.noTone();
    arduboy.print("SeekingUP");
    moveCircle_L_R();
  }
  if (arduboy.pressed(LEFT_BUTTON)) {
    radio.seekDown();
    sound.tone(2000);
    delay(50); //was delay(100);
    sound.noTone();
    arduboy.setCursor(36, 50);
    arduboy.print("SeekingDOWN");
    moveCircle_R_L();
  }
  /*
    if (arduboy.pressed(A_BUTTON)) {
    //delay(BUTTON_DELAY);
    //radio.setMute(true);
    radio.setBassBoost(true);
    sound.tone(500);
    arduboy.setCursor(40, 50);
    //arduboy.print("RadioMute");
    arduboy.print("BassBoostON");
    delay(200);
    sound.noTone();
    moveCircle();
    }
    if (arduboy.pressed(B_BUTTON)) {
    //delay(BUTTON_DELAY);
    //radio.setMute(false);
    radio.setBassBoost(false);
    sound.tone(800);
    arduboy.setCursor(38, 50);
    //arduboy.print("RadioUnMute");
    arduboy.print("BassBoostOFF");
    delay(200);
    sound.noTone();
    moveCircle();
    }
  */
  if (arduboy.pressed(A_BUTTON)) {
    //delay(BUTTON_DELAY);
    //radio.setMute(true);
    //radio.setBassBoost(true);
    arduboy.displayOn();
    sound.tone(500);
    arduboy.setCursor(40, 50);
    arduboy.print("DisplayON");
    //arduboy.print("BassBoostON");
    delay(100);
    sound.noTone();
    moveCircle_L_R();
  }

  if (arduboy.pressed(B_BUTTON)) {
    //delay(BUTTON_DELAY);
    //radio.setMute(false);
    //radio.setBassBoost(false);
    sound.tone(800);
    arduboy.setCursor(38, 50);
    arduboy.print("DisplayOFF");
    //arduboy.print("BassBoostOFF");
    delay(100);
    sound.noTone();
    moveCircle_L_R();
    arduboy.displayOff();
  }
  //}
  //arduboy.setCursor(30, 22);
  arduboy.setCursor(30, 1);
  arduboy.print("rssi- ");
  arduboy.print(radio.getRSSI());
  arduboy.print(" dBuV");
  /*
    arduboy.setCursor(40, 30);
    frequency = radio.getFrequency();
    //arduboy.print("Currently tuned to ");
    arduboy.print(frequency / 100);
    arduboy.print(".");
    arduboy.print(frequency % 100);
    arduboy.setCursor(80, 32);
    arduboy.print("MHz");
  */
  // check for RDS data
  radio.checkRDS();

  // update the display from time to time
  if (now > nextFreqTime) {
    f = radio.getFrequency();
    if (f != lastf) {
      // print current tuned frequency
      DisplayFrequency(f);
      lastf = f;
    } // if
    nextFreqTime = now + 400;
  } // if


  autoPowerDown();
}

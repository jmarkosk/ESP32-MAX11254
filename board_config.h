/**
  ******************************************************************************
  * @file    max11210.h 
  * @author  Domen Jurkovic
  * @version V1.0
  * @date    18-Nov-2015
  * @brief   Header for max11210.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#define BOARD_BANGOOD_DEVKIT
//#define BOARD_ESPRESSIF_WROVER_DEVKIT



#ifdef BOARD_BANGOOD_DEVKIT
 #define OUTPUT_LED_RED   14 
 #define OUTPUT_LED_GREEN 12 
 #define OUTPUT_LED_BLUE  13
 #define BOOT_PIN          0
 #define BLINK_GPIO OUTPUT_LED_RED
 #define LED_PIN OUTPUT_LED_BLUE
 #define BUTTON_PIN BOOT_PIN
 #define csPin   5 // csPin
 #define clkPin  18// CLK
 #define mosiPin 23// MOSI
 #define misoPin 19 // MISO
 #define MAX11254_RSTB_PIN    2 //Active low power on reset input
 #define MAX11254_RDYB_PIN     4 //Active low
#endif

#ifdef BOARD_ESPRESSIF_WROVER_DEVKIT
#define csPin   15 // csPin
#define clkPin  14// CLK
#define mosiPin 13// MOSI
#define misoPin 12 // MISO
#endif

#endif

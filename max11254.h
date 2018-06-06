/**
  ******************************************************************************
  * @file    MAX11254.h 
  * @author  Domen Jurkovic
  * @version V1.0
  * @date    18-Nov-2015
  * @brief   Header for MAX11254.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAX11254_H
#define __MAX11254_H
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "board_config.h"

/* Includes ------------------------------------------------------------------*/
//#include "stm32l1xx.h"

//#include "stm32l1xx_gpio_init.h"
//#include "systick_millis.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//#define MAX11210
#define MAX11254

#ifdef MAX11210
  #define	MAX11210_SPI_BANK		NotUSED
  #define	MAX11210_SPI_NSS		csPin
  #define	MAX11210_SPI_SCK		clkPin
  #define	MAX11210_SPI_MISO		misoPin
  #define	MAX11210_SPI_MOSI		mosiPin  
#endif

#ifdef MAX11254
  #define	MAX11254_SPI_BANK		NotUSED
  #define	MAX11254_SPI_NSS		csPin
  #define	MAX11254_SPI_SCK		clkPin
  #define	MAX11254_SPI_MISO		misoPin
  #define	MAX11254_SPI_MOSI		mosiPin
#endif

/*
#define	AD_GPIO1_BANK				NotUSED
#define	AD_GPIO1						GPIO_Pin_0
#define	AD_GPIO2						GPIO_Pin_1	// uncomment in MAX11210_init()
#define	AD_GPIO3						GPIO_Pin_2	// uncomment in MAX11210_init()
#define	AD_GPIO4						GPIO_Pin_3	// uncomment in MAX11210_init()
*/

// REGISTER ADDRESS
#ifdef MAX11210
  #define	STAT1	0x0	// reg size = 8 bits
  #define	CTRL1	0x1	// reg size = 8 bits
  #define	CTRL2	0x2	// reg size = 8 bits (FOR GPIO pins)
  #define	CTRL3	0x3	// reg size = 8 bits
  #define	DATA	0x4	// reg size = 24 bits
  #define	SOC		0x5	// reg size = 24 bits		(System Offset Calibration)
  #define	SGC		0x6	// reg size = 24 bits		(System Gain Calibration)
  #define	SCOC	0x7	// reg size = 24 bits		(Self-calibration Offset)
  #define	SCGC	0x8	// reg size = 24 bits		(Self-calibration Gain)
#endif

#ifdef MAX11254
    #define	STAT1	    0x0	// reg size = 8 bits
    #define	CTRL1	    0x1	// reg size = 8 bits
    #define	CTRL2	    0x2	// reg size = 8 bits (FOR GPIO pins)
    #define	CTRL3	    0x3	// reg size = 8 bits
    #define GPIO_CTRL 0x4
    #define DELAY     0x5
    #define CHMAP1    0X6
    #define CHMAP0    0x7
    #define SEQ       0x8
    #define GPO_DIR   0x9          
    #define	SOC		    0x10	// reg size = 24 bits		(System Offset Calibration)
    #define	SGC		    0x11// reg size = 24 bits		(System Gain Calibration)
    #define	SCOC	    0x12	// reg size = 24 bits		(Self-calibration Offset)
    #define	SCGC	    0x13	// reg size = 24 bits		(Self-calibration Gain)
    #define DATA0     0x14
    #define DATA1     0x15
    #define DATA2     0x16
    #define DATA3     0x17
    #define DATA4     0x18
    #define DATA5     0x19

    // COMMAND BYTES  (MODE = 0, LINEF = 0)
    //CTRL1 : Control Register 1
    //This are all masked.  Need to read then 'AND' with register
   //Calibration
    #define	SELF_CALIB				0x3f  // [Bit7: 0, Bit6: 0, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    #define	SLOC_CALIB				0x7f  // [Bit7: 0, Bit6: 1, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    #define	SYSF_CALIB				0xbf  // [Bit7: 1, Bit6: 0, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    #define	DNU_CALIB 				0xff  // [Bit7: 1, Bit6: 1, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x] DO NOT USE
    //Power state
    #define	POWER_NOP 				0xcf  // [Bit7: x, Bit6: x, Bit5: 0, Bit4: 0, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    #define	POWER_SLEEP 		  0xdf  // [Bit7: x, Bit6: x, Bit5: 0, Bit4: 1, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    #define	POWER_STANDBY 		0xef  // [Bit7: x, Bit6: x, Bit5: 1, Bit4: 0, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    #define	POWER_RESET 			0xfF  // [Bit7: x, Bit6: x, Bit5: 1, Bit4: 1, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
    //Conversion type
    #define CONN_POLAR_TYPE   0xF7  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: 0, Bit2: x, Bit1: x, Bit0: x] 
    //Format
    #define DATA_FORMAT       0xfb  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: 0, Bit1: x, Bit0: x]
    //Conversion scycle type: Single vs Continuous
    #define CONV_SCYCLE       0xfd  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: 0, Bit0: x]
    //Conversion scycle type: Single vs Continuous
    #define CONV_CONTSC       0xfe  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: 0]

    //***********************************************************************************************************
    //CTRL2 : Control Register 2
    //This are all masked.  

    //Clock Mode
     #define EXTCLK_MODE         0x00 // [Bit7: 0, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: 0]
    //External clock: 0x80
    //Internal clock: 0x00
    #define CCSEN 0x40
    //CCSEN: 0x40 enables current source current sink on the analog inputs to detect sensor opens and shorts
    #define LDOEN 0x20
    //Set '1' to enable internal LDO
    #define LPMODE      0x10
    #define PGAEN       0x80
    #define PGA_GAIN1   0x00
    #define PGA_GAIN2   0x01
    #define PGA_GAIN4   0x02
    #define PGA_GAIN8   0x03
    #define PGA_GAIN16  0x04
    #define PGA_GAIN32  0x05
    #define PGA_GAIN64  0x06
    #define PGA_GAIN128 0x07
    //***********************************************************************************************************
    //CTRL3 : Control Register 3
    //This are all masked. 
    #define GPO_MODE  6
    #define SYNC_MODE 5
    #define CALREGSEL 4
    #define NOSYSG    3
    #define NOSYSO    2
    #define NOSCG     1
    #define NOSCO     0
    //***********************************************************************************************************
    //GPIO_CTRL: GPIO Control Register
    #define GPIO_CTRL_REG 0x40
    //  #define	SYS_OFFSET_CALIB	0xA0
    //  #define	SYS_GAIN_CALIB		0xB0
    #define	MEASURE_1_SPS			0x80
    #define	MEASURE_2p5_SPS		0x81
    #define	MEASURE_5_SPS			0x82
    #define	MEASURE_10_SPS		0x83
    #define	MEASURE_15_SPS		0x84
    #define	MEASURE_30_SPS		0x85
    #define	MEASURE_60_SPS		0x86
    #define	MEASURE_120_SPS		0x87
//***********************************************************************************************************
    
    #define CONVERSION_COMMAND 0x80
    #define REGISTER_RW        0xc0
    
    
    //Command BYTE
    //MODE[1:0]
    #define MODE_UNUSED      0x00  
    #define MODE_POWER_DOWN  0x10
    #define MODE_CALIBRATION 0x20
    #define MODE_SEQUENCER   0x30
    //RATE[3:0]
    //RS[4:0]
    //R/W





#endif

#ifdef MAX11210
  #define SELF_CALIB 0x00 //CTRL 1
  // MEASUREMENT MODE
  #define MODE_SINGLE				0x02
  #define MODE_CONTINUOUS		0x00

  //ADC WAIT TIMEOUT
    //this is in seconds
#endif

#define MAX_TIME_TO_WAIT 5

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SPI_init(void);
void MAX11254_init(void);
void read_registers(void);

void MAX11254_calibration(void);
void MAX11254_self_calib(void);
void MAX11254_sys_offset_calib(void);
void MAX11254_sys_gain_calib(void);

void			MAX11254_set_meas_mode(uint8_t mode);
void 			MAX11254_start_meas(uint8_t rate);
uint32_t	MAX11254_read_result(void);
uint8_t 	MAX11254_meas_status(void);


uint32_t   	MAX11254_read_reg(uint8_t reg);
void  			MAX11254_write_reg(uint8_t reg, uint8_t reg_val_HSB, uint8_t reg_val_MSB, uint8_t reg_val_LSB);
//void        MAX11254_write_reg(uint8_t reg, uint8_t reg_val_HSB, uint8_t reg_val_MSB, uint8_t reg_val_LSB, uint8_t length);
//void 			MAX11254_send_command(uint8_t command);
void        MAX11254_send_command(uint8_t command, uint8_t length);






#endif /* __MAX11254_H */

/*****	END OF FILE	****/

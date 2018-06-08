/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "max11254.h"
#include "board_config.h"

#include "sdkconfig.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

#define SHIFT	5
#define AV_RATE	10
#define __IO_INTERRUPT__


static const char* tag = "BLINK";
static const char* tag1 = "MAX11254";
static const char* task_console = "CONSOLE";


SemaphoreHandle_t xSemaphore = NULL;

void read_registers(void);
static void blink_task(void *pvParameters);
static void spi_task(void *pvParameters);
static void console_task(void *pvParameters);



void read_registers(void){

    printf("\nSTAT1: 0x%08x\n",MAX11254_read_reg(STAT1));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("CTRL1:  0x%08x\n",MAX11254_read_reg(CTRL1));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("CTRL2: 0x%08x\n",MAX11254_read_reg(CTRL2));
    vTaskDelay(100 / portTICK_PERIOD_MS);   
    printf("CTRL3: 0x%08x\n",MAX11254_read_reg(CTRL3));	
    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("SCOC: 0x%08x\n",MAX11254_read_reg(SCOC));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("SOC:  0x%08x\n",MAX11254_read_reg(SOC));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("SCGC: 0x%08x\n",MAX11254_read_reg(SCGC));
    vTaskDelay(100 / portTICK_PERIOD_MS);   
    printf("SGC: 0x%08x\n\n",MAX11254_read_reg(SGC));	
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
}




void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    uint8_t j = 0;
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    for(;;) {
        /* Blink off (output low) */
        //xSemaphoreTake(xMutex, portMAX_DELAY);
              
        //xSemaphoreGive(xMutex);
        gpio_set_level(BLINK_GPIO, 0);        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
       
    }
}

void spi_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    
    int32_t result,result2;
	int32_t av_result;
    int32_t av_result2;
	uint8_t i = 0;

    static uint8_t count =0;   
    
    gpio_pad_select_gpio(MAX11254_RSTB_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(MAX11254_RSTB_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MAX11254_RSTB_PIN, 1);
    ESP_LOGI(tag1, ">> inializing");

    MAX11254_init();    
    ESP_LOGI(tag1, ">> initialized");     
/*
    for(i = 0; i < 5; i ++){
		MAX11254_start_meas(MEASURE_10_SPS);
		MAX11254_read_result();
	}
*/	
 //   ESP_LOGI(tag1, "... Calibration Start");   
 //   MAX11254_calibration();
 //   ESP_LOGI(tag1, "... Calibration END");   


    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("SCOC: 0x%08x\n",MAX11254_read_reg(SCOC));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("SOC:  0x%08x\n",MAX11254_read_reg(SOC));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("SCGC: 0x%08x\n",MAX11254_read_reg(SCGC));
    vTaskDelay(300 / portTICK_PERIOD_MS);   
    printf("SGC: 0x%08x\n\n",MAX11254_read_reg(SGC));	
    vTaskDelay(300 / portTICK_PERIOD_MS);


   // ESP_LOGI(tag1, "... Self Calibration Start");   
   // MAX11254_self_calib();
   // ESP_LOGI(tag1, "... Self Calibration End\n\n");   

    printf("SCOC: 0x%08x\n",MAX11254_read_reg(SCOC));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("SOC:  0x%08x\n",MAX11254_read_reg(SOC));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("SCGC: 0x%08x\n",MAX11254_read_reg(SCGC));
    vTaskDelay(300 / portTICK_PERIOD_MS);   
    printf("SGC: 0x%08x\n\n",MAX11254_read_reg(SGC));	
    vTaskDelay(300 / portTICK_PERIOD_MS);

    printf("STAT1: 0x%08x\n",MAX11254_read_reg(STAT1));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("CTRL1:  0x%08x\n",MAX11254_read_reg(CTRL1));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("CTRL2: 0x%08x\n",MAX11254_read_reg(CTRL2));
    vTaskDelay(300 / portTICK_PERIOD_MS);   
    printf("CTRL3: 0x%08x\n",MAX11254_read_reg(CTRL3));	
    vTaskDelay(300 / portTICK_PERIOD_MS);

    printf("*************\n\n");    
    printf("CTRL1:  0x%08x\n",MAX11254_read_reg(CTRL1));
    printf("*************\n\n");

    for(;;)  
    {

        if(xSemaphore != NULL)
        {
            if(xSemaphoreTake (xSemaphore, (TickType_t) (10 / portTICK_PERIOD_MS)) == pdTRUE)
            {

                for(i = 0; i < AV_RATE; i++){
		            MAX11254_start_meas(MEASURE_10_SPS);	//starts a single conversion
		            result = MAX11254_read_result() & 0x00FFFFFF;
                    result2 = result;
		            if((result >> 23) == 0){
		            }
		            else{		//negativna cifra
				        result = result << 8;
				        result  = ~result + 1;
				        result  = result >> 8;				
				        result = -result;
		            }
			        av_result += result;
                    av_result2 += result2;
			        result=0;
                    result2=0;
		        }
		
		        av_result = av_result / AV_RATE ;
                av_result2 = av_result2 / AV_RATE ;
		        //printNumberLn(av_result >> SHIFT, DEC);
                printf("av_result: 0x%08x\n",(av_result >> SHIFT));
                printf("av_result2: 0x%08x\n",av_result2);
		        av_result = 0;
                av_result2 = 0;
                count++;
                xSemaphoreGive(xSemaphore);
            }
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);             
    }      

}


/* TODO : ADD EVENT DRIVEN UART CODE -- POLLING VERY SLOW */

void console_task(void *pvParameter)
{
    
    uint8_t j = 0;

    for(;;){

        if(xSemaphore != NULL)
        {
            
            if(xSemaphoreTake (xSemaphore, (TickType_t) (10 / portTICK_PERIOD_MS)) == pdTRUE)
            {
                //ESP_LOGD(task_console, "xSemaphoreTake worked!!");
                vTaskDelay(200 / portTICK_PERIOD_MS);
                j = getchar();
                if(j == 'a')ESP_LOGD(task_console, "getchar worked!!");
                if(j == 's'){
                    ESP_LOGD(task_console, "self_calibrate...");
                    MAX11254_self_calib();
                }    
                if(j == 'r'){
                    ESP_LOGD(task_console, "read registers...");
                    read_registers();
                }
                if(j == 'w'){
                    ESP_LOGD(task_console, "Write CTRL3 0x00...");
                    MAX11254_write_reg(CTRL3,0x00,0,0);
                }       
                xSemaphoreGive(xSemaphore);
            }
           // else ESP_LOGE(task_console, "xSemaphoreTake failed!!");

        } 
    
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

}



#ifdef __IO_INTERRUPT__

#define ESP_INTR_FLAG_DEFAULT 0

SemaphoreHandle_t xSemaphoreGPIO = NULL;
bool led_status = false;

// interrupt service routine, called when the button is pressed
void IRAM_ATTR button_isr_handler(void* arg) {
	
    // notify the button task
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_DISABLE);
	xSemaphoreGiveFromISR(xSemaphoreGPIO, NULL);
}

// task that will react to button clicks
void button_task(void* arg) {
	
	// infinite loop
	for(;;) {
		
		// wait for the notification from the ISR
		if(xSemaphoreTake(xSemaphoreGPIO,portMAX_DELAY) == pdTRUE) {
			printf("Button pressed!\n");
			led_status = !led_status;
			gpio_set_level(LED_PIN, led_status);
            vTaskDelay(10 / portTICK_PERIOD_MS); //10ms delay for debounce
            gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
		}
	}
}
#endif



//#define ESP_INTR_FLAG_DEFAULT 0

SemaphoreHandle_t xSemaphoreGPIO_RDY = NULL;


// interrupt service routine, called when the button is pressed
void IRAM_ATTR MAX11254_conversion_complete_isr_handler(void* arg) {
	
    // notify the button task
    gpio_set_intr_type(MAX11254_RDYB_PIN, GPIO_INTR_DISABLE);
	xSemaphoreGiveFromISR(xSemaphoreGPIO_RDY, NULL);
}


void MAX11254_conversion_complete_task(void* arg){

    for(;;){

        if(xSemaphoreTake(xSemaphoreGPIO_RDY,portMAX_DELAY) == pdTRUE) {
			printf("Converstion Complete!\n");			
            vTaskDelay(10 / portTICK_PERIOD_MS); //10ms delay for debounce
            gpio_set_intr_type(MAX11254_RDYB_PIN, GPIO_INTR_NEGEDGE);
		}

    }
}


void app_main()
{
    
 #ifdef __IO_INTERRUPT__

	// create the binary semaphore
	xSemaphoreGPIO = xSemaphoreCreateBinary();
	
	// configure button and led pins as GPIO pins
	gpio_pad_select_gpio(BUTTON_PIN);
	gpio_pad_select_gpio(LED_PIN);
	
	// set the correct direction
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	
	// enable interrupt on falling (1->0) edge for button pin
	gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
	
	
	
	// install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	// attach the interrupt service routine
	gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);


 #endif  

 	// create the binary semaphore
	xSemaphoreGPIO_RDY = xSemaphoreCreateBinary();
	
	// configure button and led pins as GPIO pins
	gpio_pad_select_gpio(MAX11254_RDYB_PIN);
	
	
	// set the correct direction
	gpio_set_direction(MAX11254_RDYB_PIN, GPIO_MODE_INPUT);
    
	
	// enable interrupt on falling (1->0) edge for button pin
	gpio_set_intr_type(MAX11254_RDYB_PIN, GPIO_INTR_NEGEDGE);
	
	
	
	// install ISR service with default configuration
	//gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	// attach the interrupt service routine
	gpio_isr_handler_add(MAX11254_RDYB_PIN, MAX11254_conversion_complete_isr_handler, NULL);
   
   
   
   
   
   
    xSemaphore = xSemaphoreCreateMutex();
    // vSemaphoreCreateBinary( xSemaphore );
    xTaskCreate(&blink_task,               //Pointer to the function of the task (function pointer)
    		    "blink_task",              //Debug task name
				 2048, //Stack size
				 NULL,                     //pointer to task parameters
				 10,                        //task priority
				 NULL);                    //task handel


    xTaskCreate(&spi_task,               //Pointer to the function of the task (function pointer)
    		    "spi_task",              //Debug task name
				 2048,                   //Stack size
				 NULL,                   //pointer to task parameters
				 5,                      //task priority
				 NULL);                  //task handel

    xTaskCreate(&console_task,           //Pointer to the function of the task (function pointer)
    		    "console_task",          //Debug task name
				 2048,                   //Stack size
				 NULL,                   //pointer to task parameters
				 4,                      //task priority
				 NULL);                  //task handel


// start the task that will handle the button
	xTaskCreate(&MAX11254_conversion_complete_task,             //Pointer to the function of the task (function pointer)
                 "MAX11254_conversion_complete_task",          //Debug task name
                 2048,                   //Stack size
                 NULL,                   //pointer to task parameters
                 10,                     //task priority
                 NULL);                  //task handel

    xTaskCreate(&button_task,
                "button_task",
                2048,
                NULL,
                10,
                NULL);             


}

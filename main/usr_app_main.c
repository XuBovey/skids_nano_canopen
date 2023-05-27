#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdlib.h>
#include <stdint.h>
#include "driver/can.h"
#include "esp_timer.h"
#include "soc/soc.h"
#include "CANopen.h"
#include "CO_OD.h"
#include "CO_config.h"
#include "modul_config.h"
#include "pmc.h"
// #include "dunker.h"
// #include "Gyro.h"

uint8_t counter = 0;

volatile uint32_t coInterruptCounter = 0U; /* variable increments each millisecond */

//Timer Interrupt Configuration
static void coMainTask(void *arg);

esp_timer_create_args_t coMainTaskArgs;
//Timer Handle
esp_timer_handle_t periodicTimer;

void mainTask(void *pvParameter)
{
    coMainTaskArgs.callback = &coMainTask;
    coMainTaskArgs.name = "coMainTask";
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;

    vTaskDelay(BOOT_WAIT / portTICK_PERIOD_MS);

    while (reset != CO_RESET_APP)
    {
        /* CANopen communication reset - initialize CANopen objects *******************/
        CO_ReturnError_t err;
        uint32_t coInterruptCounterPrevious;

        /* initialize CANopen */
        err = CO_init(NULL, NODE_ID_MASTER /* NodeID */, CAN_BITRATE /* bit rate */);
        if (err != CO_ERROR_NO)
        {
            ESP_LOGE("mainTask", "CO_init failed. Errorcode: %d", err);
            CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
            esp_restart();
        }
        /* Configure Timer interrupt function for execution every CO_MAIN_TASK_INTERVAL */
        ESP_ERROR_CHECK(esp_timer_create(&coMainTaskArgs, &periodicTimer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodicTimer, CO_MAIN_TASK_INTERVAL));

        /* start CAN */
        ESP_LOGI("mainTask", "start can ctrl.");
        CO_CANsetNormalMode(CO->CANmodule[0]);

        reset = CO_RESET_NOT;
        coInterruptCounterPrevious = coInterruptCounter;

        // CO_sendNMTcommand(CO, CO_NMT_ENTER_PRE_OPERATIONAL, NODE_ID_PMC0);
		ESP_LOGI("mainTask", "set CO_NMT_ENTER_PRE_OPERATIONAL.");
        CO_sendNMTcommand(CO, CO_NMT_ENTER_PRE_OPERATIONAL, 0x00);
        // vTaskDelay(MAIN_WAIT / portTICK_PERIOD_MS);
        ESP_LOGI("mainTask", "pmc_init."); 
        pmc_init(CO, NODE_ID_PMC0, 0);

        /*Set Operating Mode of Slaves to Operational*/
        // vTaskDelay(MAIN_WAIT / portTICK_PERIOD_MS);
        ESP_LOGI("mainTask", "set motor enter operational.");
        // CO_sendNMTcommand(CO, CO_NMT_ENTER_OPERATIONAL, NODE_ID_PMC0);
        CO_sendNMTcommand(CO, CO_NMT_ENTER_OPERATIONAL, 0x00);
        // ESP_LOGI("mainTask", "mode = %d.", CO->NMT->operatingState);

        /* Initialise system components */

        /* application init code goes here. */
		ESP_LOGI("mainTask", "loop.");
	
#define PDO_TEST 1

#ifndef PDO_TEST
        while(1)
        {
            pmc_sdoMotorTest();
            vTaskDelay(MAIN_WAIT / portTICK_PERIOD_MS);
        }
#endif
#ifdef PDO_TEST
        while (reset == CO_RESET_NOT)
        {
            uint32_t coInterruptCounterCopy;
            uint32_t coInterruptCounterDiff;
            int32_t setSpeed = 0;
            /* loop for normal program execution ******************************************/

            coInterruptCounterCopy = coInterruptCounter;
            coInterruptCounterDiff = coInterruptCounterCopy - coInterruptCounterPrevious;
            coInterruptCounterPrevious = coInterruptCounterCopy;

            // if(CO->NMT->operatingState != 5){
            //     ESP_LOGI("mainTask", "set self enter operational, current = %d.", CO->NMT->operatingState);
            //     CO_sendNMTcommand(CO, CO_NMT_ENTER_OPERATIONAL, NODE_ID_MASTER);
            // }

            /* CANopen process */
			// ESP_LOGI("mainTask", "OD_errorRegister = 0x%x.", OD_errorRegister);
            reset = CO_process(CO, coInterruptCounterDiff, NULL);			

            /* Nonblocking application code may go here. */
            if (counter == 0)
            {
                setSpeed = 20000;
                ESP_LOGI("mainTask", "pmc motor set setSpeed %d.", setSpeed);
                pmc_setSpeed(setSpeed);
                counter++;
            }
            if (coInterruptCounter > 10 && counter == 1)
            {
                setSpeed = 50000;
                ESP_LOGI("mainTask", "pmc motor set setSpeed %d.", setSpeed);
                pmc_setSpeed(setSpeed);
                counter++;
            }
            if (coInterruptCounter > 20 && counter == 2)
            {
                setSpeed = 80000;
                ESP_LOGI("mainTask", "pmc motor set setSpeed %d.", setSpeed);
                pmc_setSpeed(setSpeed);
                counter++;
            }
            if (coInterruptCounter > 30 && counter == 3)
            {
                setSpeed = 100000;
                ESP_LOGI("mainTask", "pmc motor set setSpeed %d.", setSpeed);
                pmc_setSpeed(setSpeed);
                counter++;
            }
            if (coInterruptCounter > 80 && counter == 4)
            {
                setSpeed = 150000;
                ESP_LOGI("mainTask", "pmc motor set setSpeed %d.", setSpeed);
                pmc_setSpeed(setSpeed);
                counter++;
            }
            if (coInterruptCounter > 150 && counter == 5)
            {
                ESP_LOGI("mainTask", "pmc motor halt.");
                pmc_quickStop();
                counter++;
                coInterruptCounter = 0;
            }
            if (coInterruptCounter > 200 && counter == 6)
            {
                ESP_LOGI("mainTask", "pmc motor halt.");
                pmc_quickStop();
                counter++;
                coInterruptCounter = 0;
            }
            // ESP_LOGI("mainTask", "pmc setSpeed = %d.", setSpeed);
#endif
            if(counter == 7)
                counter = 0;
            /* Wait */
            vTaskDelay(MAIN_WAIT / portTICK_PERIOD_MS);
            coInterruptCounter += 1;
        }
            
    }
    /* program exit
        * ***************************************************************/
    /* reset */
    esp_restart();
}

/* CanOpen-Task executes in constant intervals ********************************/
static void coMainTask(void *arg)
{
    if (CO->CANmodule[0]->CANnormal)
    {
        bool_t syncWas = true;

        /* Process Sync */
        syncWas = CO_process_SYNC(CO, CO_MAIN_TASK_INTERVAL);

        /* Read inputs */
        CO_process_RPDO(CO, syncWas);

        /* Write outputs */
        CO_process_TPDO(CO, syncWas, CO_MAIN_TASK_INTERVAL);
    }
}

void app_main()
{
    xTaskCreate(&mainTask, "mainTask", 4096, NULL, 5, NULL);
}

#include "pmc.h"
#include "modul_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "CANopen.h"
#include "CO_OD.h"
#include "esp_log.h"

// static motorRegister motor[MAX_MOTORS];

static CO_t *_CO = NULL;
static uint8_t _nodeId = 0x00;
static uint8_t _tpdoNum = 0x00;

uint8_t OD_pusi_moto_Status;
uint8_t OD_pusi_moto_driver_status;
uint8_t OD_pusi_moto_mode;
uint8_t OD_pusi_moto_stop;
// uint8_t *power;
int32_t OD_pusi_moto_speed;
int32_t OD_pusi_moto_postion;

/**
 * @brief Array for all Motors and the corresponding object-dictionary entries
 *
 */
static motorRegister motor[] =
{{	&OD_pusi_moto_Status, 
	&OD_pusi_moto_driver_status, 
	&OD_pusi_moto_mode, 
	&OD_pusi_moto_stop, 
	&OD_pusi_moto_speed, 
	&OD_pusi_moto_postion}};

/**
 * @brief Number of created objects
 *
 */
static uint8_t numMotors = 0;

/**
 * @brief Motornumber of this object
 *
 */
uint8_t motorNumber = 0;

void pmc(void)
{
		motorNumber = numMotors;
		numMotors++;
}

int8_t pmc_init(CO_t *CO, uint8_t nodeId, uint8_t tpdoNum)
{
		int8_t ret = 0;
		_CO = CO;
		_nodeId = nodeId;
		_tpdoNum = tpdoNum;
		/*Configure PDO Mapping on Device pmc MOTOR*/
		#if 0
		uint32_t mappedRxObjects[] = {0x40000108, 0x40030108, 0x40040108, 0x43000120};
		ret = pmc_mapRPDO(0, _nodeId, mappedRxObjects, 4);
		uint32_t mappedTxObjects[] = {0x40020120, 0x40010110};
		ret += pmc_mapTPDO(0, _nodeId, mappedTxObjects, 2, 0x100, 0x100);
		#endif
		return ret;
}

int8_t pmc_clearError(void)
{
		//*motor[motorNumber].command = CMD_ClearError;
		*motor[motorNumber].velocity = 0;
		CO->TPDO[_tpdoNum]->sendRequest = 1;
		return 0;
}

int8_t pmc_quickStop(void)
{
		//*motor[motorNumber].command = CMD_QuickStop;
		*motor[motorNumber].stop = 0;
		*motor[motorNumber].velocity = 0;
		CO->TPDO[_tpdoNum]->sendRequest = 1;
		return 0;
}

int8_t pmc_halt(void)
{
		//*motor[motorNumber].command = CMD_Halt;
		*motor[motorNumber].stop = 0;
		*motor[motorNumber].velocity = 0;
		CO->TPDO[_tpdoNum]->sendRequest = 1;
		return 0;
}

int8_t pmc_continueMovement(void)
{
		//*motor[motorNumber].command = CMD_Continue;
		// *motor[motorNumber].velocity = 0;
		// CO->TPDO[_tpdoNum]->sendRequest = 1;
		return 0;
}

int8_t pmc_setEnable(uint8_t value)
{

		int8_t ret = 0;

		*motor[motorNumber].mode = OPERATION_MODE;
		//*motor[motorNumber].power = 1;
		CO->TPDO[_tpdoNum]->sendRequest = 1;

		/*Check for fault condition*/
		if (!(*motor[motorNumber].status & STAT_Error))
		{
				if (value)
				{
						while (!(*motor[motorNumber].status & STAT_Enabled))
						{
								vTaskDelay(1 / portTICK_PERIOD_MS);
								*motor[motorNumber].mode = OPERATION_MODE;
								//*motor[motorNumber].power = value;
								CO->TPDO[_tpdoNum]->sendRequest = 1;
						}
				}
				else
				{
						*motor[motorNumber].mode = OPERATION_MODE;
						//*motor[motorNumber].power = value;
						CO->TPDO[_tpdoNum]->sendRequest = 1;
				}
		}
		else
		{
				ret = *motor[motorNumber].error; //Can't dis-/enable motor while in fault condition
				ESP_LOGE("Pmc.setEnable", "Can't dis-/enable motor while in fault condition!");
		}
		return ret;
}

int8_t pmc_setSpeed(int32_t speed)
{
		int8_t ret = 0;

		/*Check for fault condition*/
		if (!(*motor[motorNumber].status & STAT_Error))
		{
				*motor[motorNumber].velocity = speed;
				CO->TPDO[_tpdoNum]->sendRequest = 1;
		}
		else
		{
				ret = *motor[motorNumber].error; //Can't set speed while in fault condition
		}
		return ret;
}

int8_t pmc_coProcessUploadSDO(void)
{
		uint32_t SdoAbortCode = CO_SDO_AB_NONE;
		int8_t ret = 0;
		uint32_t dataSize = 0;

		do
		{
				ret = CO_SDOclientUpload(_CO->SDOclient[0], 1, 5000, &dataSize, &SdoAbortCode);

		} while (ret > 0);
		return ret;
}

int8_t pmc_coProcessDownloadSDO(void)
{
		uint32_t SdoAbortCode = CO_SDO_AB_NONE;
		int8_t ret = 0;
		do
		{
				ret = CO_SDOclientDownload(_CO->SDOclient[0], 1, 5000, &SdoAbortCode);
		} while (ret > 0);
		return ret;
}

int8_t pmc_mapRPDO(uint8_t pdoNumber, uint8_t nodeId, uint32_t *mappedObjects, uint8_t numMappedObjects)
{
		int8_t ret = 0;
		uint32_t v32 = 0; //Temporary Storage
		uint8_t v8 = 0; //Temporary Storage

		//RPDO Disable
		v32 = ((0x200 + nodeId + pdoNumber) | 0x80000000);
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1400 + pdoNumber, 1, (uint8_t *)&v32, sizeof(v32), 0);
		ret = pmc_coProcessDownloadSDO();

		//RPDO Disable Mapping
		v8 = 0;
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1600 + pdoNumber, 0, (uint8_t *)&v8, sizeof(v8), 0);
		ret = pmc_coProcessDownloadSDO();

		//RPDO Mapping
		for (uint8_t i = 0; i < numMappedObjects; i++)
		{
				v32 = mappedObjects[i];
				CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1600 + pdoNumber, 1 + i, (uint8_t *)&v32, sizeof(v32), 0);
				ret = pmc_coProcessDownloadSDO();
		}

		//RPDO Enable Mapping
		v8 = numMappedObjects;
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1600 + pdoNumber, 0, (uint8_t *)&v8, sizeof(v8), 0);
		ret = pmc_coProcessDownloadSDO();

		//RPDO Enable
		v32 = (0x200 + nodeId + pdoNumber);
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1400 + pdoNumber, 1, (uint8_t *)&v32, sizeof(v32), 0);
		ret = pmc_coProcessDownloadSDO();

		return ret;
}

int8_t pmc_mapTPDO(uint8_t pdoNumber, uint8_t nodeId, uint32_t *mappedObjects, uint8_t numMappedObjects, uint16_t eventTime, uint16_t inhibitTime)
{
		int8_t ret = 0;
		uint32_t v32 = 0;
		uint16_t v16 = 0;
		uint8_t v8 = 0;

		//TPDO Disable
		v32 = ((0x180 + nodeId + pdoNumber) | 0x80000000);
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1800 + pdoNumber, 1, (uint8_t *)&v32, sizeof(v32), 0);
		ret = pmc_coProcessDownloadSDO();

		//TPDO Disable Mapping
		v8 = 0;
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1a00 + pdoNumber, 0, (uint8_t *)&v8, sizeof(v8), 0);
		ret = pmc_coProcessDownloadSDO();

		//TPDO Set Eventtime
		v16 = eventTime;
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1800 + pdoNumber, 5, (uint8_t *)&v16, sizeof(v16), 0);
		ret = pmc_coProcessDownloadSDO();

		//TPDO Set Inhibittime
		v16 = inhibitTime;
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1800 + pdoNumber, 3, (uint8_t *)&v16, sizeof(v16), 0);
		ret = pmc_coProcessDownloadSDO();

		//TPDO Mapping
		for (uint8_t i = 0; i < numMappedObjects; i++)
		{
				v32 = mappedObjects[i];
				CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1a00 + pdoNumber, 1 + i, (uint8_t *)&v32, sizeof(v32), 0);
				ret = pmc_coProcessDownloadSDO();
		}

		//TPDO Enable Mapping
		v8 = numMappedObjects;
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1a00 + pdoNumber, 0, (uint8_t *)&v8, sizeof(v8), 0);
		ret = pmc_coProcessDownloadSDO();

		//TPDO Enable
		v32 = (0x180 + nodeId + pdoNumber);
		CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x1800 + pdoNumber, 1, (uint8_t *)&v32, sizeof(v32), 0);
		ret = pmc_coProcessDownloadSDO();

		return ret;
}


void pmc_move_steps(uint32_t steps)
{
	int8_t ret;
	// move to
	ESP_LOGI("pmc", "move %d steps.", steps);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6004, 0, (uint8_t *)&steps, sizeof(steps), 0);
	ret = pmc_coProcessDownloadSDO();
}

void pmc_motor_test(void)
{
	int8_t ret = 0;
	uint32_t v32 = 0;
	uint8_t v8 = 0;
	uint8_t v8_rx_buf[4];
	uint32_t index = 0x600e;

	// clear error status
	ESP_LOGI("pmc", "clear error status.");
	index = 0x6012;
	v8 = 0xff;
	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)v8_rx_buf, 4, 0);
	// pmc_coProcessUploadSDO();

	// motor set to a+
	index = 0x6008;
	v8 = 5;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v8, sizeof(v8), 0);

	// motor set to a-
	index = 0x6009;
	v8 = 5;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v8, sizeof(v8), 0);

	// motor set to postion mode
	ESP_LOGI("pmc", "motor set to postion mode.");
	index = 0x6005;
	v8 = 0;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v8, sizeof(v8), 0);

	// motor set to max speed
	ESP_LOGI("pmc", "motor set to speed mode.");
	index = 0x6003;
	v32 = 20000;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v32, sizeof(v32), 0);

	// motor set to postion mode
	// ESP_LOGI("pmc", "motor set to postion mode.");
	// index = 0x6005;
	// v8 = 1;
	// CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v8, sizeof(v8), 0);

	ESP_LOGI("pmc", "motor set target postion1.");
	index = 0x6004;
	v32 = 6400;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v32, sizeof(v32), 0);
	vTaskDelay(5000 / portTICK_PERIOD_MS);

	ESP_LOGI("pmc", "motor set target postion2.");
	index = 0x6004;
	v32 = 6400;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], index, 0, (uint8_t *)&v32, sizeof(v32), 0);
	vTaskDelay(5000 / portTICK_PERIOD_MS);

	// ret = pmc_coProcessDownloadSDO();
}

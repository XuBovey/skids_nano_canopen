#include "pmc.h"
#include "modul_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "CANopen.h"
#include "CO_OD.h"
#include "esp_log.h"

static motorRegister motor[MAX_MOTORS];

static CO_t *_CO = NULL;
static uint8_t _nodeId = 0x00;
static uint8_t _tpdoNum = 0x00;

#define VERSION_STRING_LEN_MAX 16

uint8_t moto1_devName[VERSION_STRING_LEN_MAX];
uint8_t moto1_hardwareVersion[VERSION_STRING_LEN_MAX];
uint8_t moto1_softwareVersion[VERSION_STRING_LEN_MAX];
uint8_t OD_pusi_moto_Status;
uint8_t OD_pusi_moto_driver_status;
uint8_t OD_pusi_moto_mode;
uint8_t OD_pusi_moto_stop;
// uint8_t *power;
int32_t OD_pusi_moto_volcity;
int32_t OD_pusi_moto_postion;

#define TARGET_IS_DONE(target, current) (current < (target+10) && current > (target-10))	

/**
 * @brief Array for all Motors and the corresponding object-dictionary entries
 *
 */
// static motorRegister motor[MAX_MOTORS] =
// {{	&OD_pusi_moto_Status, 
// 	&OD_pusi_moto_driver_status, 
// 	&OD_pusi_moto_mode, 
// 	&OD_pusi_moto_stop, 
// 	&OD_pusi_moto_volcity, 
// 	&OD_pusi_moto_postion}};

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

void pmc_moto(void)
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
	*motor[motorNumber].errorStatus = 0xFF; // clear all error status
	*motor[motorNumber].velocity = 0;
	CO->TPDO[_tpdoNum]->sendRequest = 1;
	return 0;
}

int8_t pmc_quickStop(void)
{
	*motor[motorNumber].stop = 1; // 1-停止，0-正常
	*motor[motorNumber].velocity = 0;
	CO->TPDO[_tpdoNum]->sendRequest = 1;
	return 0;
}

int8_t pmc_halt(void)
{
	*motor[motorNumber].stop = 1; // 1-停止，0-正常
	*motor[motorNumber].velocity = 0;
	CO->TPDO[_tpdoNum]->sendRequest = 1;
	return 0;
}

int8_t pmc_continueMovement(void)
{
	*motor[motorNumber].stop = 0; // 1-停止，0-正常
	CO->TPDO[_tpdoNum]->sendRequest = 1;
	return 0;
}

int8_t pmc_setEnable(uint8_t value)
{
	int8_t ret = 0;

	*motor[motorNumber].operationMode = OPERATION_MODE;
	*motor[motorNumber].enableWr = value;
	CO->TPDO[_tpdoNum]->sendRequest = 1;

	/*Check for fault condition*/
	if (0 == *motor[motorNumber].errorStatus)
	{
		if (value)
		{
			while (*motor[motorNumber].enableRd != value)
			{
				vTaskDelay(1 / portTICK_PERIOD_MS);
				*motor[motorNumber].operationMode = OPERATION_MODE;
				*motor[motorNumber].enableWr = value;
				CO->TPDO[_tpdoNum]->sendRequest = 1;
			}
		}
		else
		{
			*motor[motorNumber].operationMode = OPERATION_MODE;
			*motor[motorNumber].enableWr = value;
			CO->TPDO[_tpdoNum]->sendRequest = 1;
		}
	}
	else
	{
		ret = *motor[motorNumber].errorStatus; //Can't dis-/enable motor while in fault condition
		ESP_LOGE("Pmc.setEnable", "Can't dis-/enable motor while in fault condition!");
	}
	return ret;
}

int8_t pmc_setSpeed(int32_t speed)
{
	int8_t ret = 0;

	/*Check for fault condition*/
	if (0 == *motor[motorNumber].errorStatus)
	{
		*motor[motorNumber].velocity = speed;
		CO->TPDO[_tpdoNum]->sendRequest = 1;
	}
	else
	{
		ret = *motor[motorNumber].errorStatus; //Can't set speed while in fault condition
		ESP_LOGE("Pmc.setEnable", "Can't set speed while in fault condition!");
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

int8_t pmc_sdoSetOperatorMode(uint8_t _mode)
{
	uint8_t mode = _mode;
	ESP_LOGI("pmc", "set motor mode %d.", mode);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6005, 0, (uint8_t *)&mode, sizeof(mode), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoMoveSteps(uint32_t _steps)
{
	uint32_t steps = _steps;
	ESP_LOGI("pmc", "move %d offset steps.", steps);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6004, 0, (uint8_t *)&steps, sizeof(steps), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoMoveToSteps(int32_t _steps)
{
	int32_t steps = _steps;
	ESP_LOGI("pmc", "move to %d steps.", steps);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x601c, 0, (uint8_t *)&steps, sizeof(steps), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoSetDirection(uint8_t _dir)
{
	uint8_t dir = _dir;
	ESP_LOGI("pmc", "set direction %d.", dir);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6002, 0, (uint8_t *)&dir, sizeof(dir), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoClearError(void)
{
	uint8_t v8 = 0xff;

	ESP_LOGI("pmc", "clear error status.");
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6012, 0, (uint8_t *)&v8, sizeof(v8), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoSetVelocity(int32_t _velocity)
{
	int32_t velocity = _velocity;
	ESP_LOGI("pmc", "set velocity %d.", velocity);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x601c, 0, (uint8_t *)&velocity, sizeof(velocity), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoSetStartVelocity(int32_t _velocity)
{
	int32_t velocity = _velocity;
	ESP_LOGI("pmc", "set velocity %d.", velocity);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6006, 0, (uint8_t *)&velocity, sizeof(velocity), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoSetStopVelocity(int32_t _velocity)
{
	int32_t velocity = _velocity;
	ESP_LOGI("pmc", "set velocity %d.", velocity);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6007, 0, (uint8_t *)&velocity, sizeof(velocity), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoSetMaxVelocity(int32_t _velocity)
{
	int32_t velocity = _velocity;
	ESP_LOGI("pmc", "set velocity %d.", velocity);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6003, 0, (uint8_t *)&velocity, sizeof(velocity), 0);
	return pmc_coProcessDownloadSDO();
}

int8_t pmc_sdoReadDeviceInfomation(void)
{
	int8_t ret;

	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x1008, 0, moto1_devName, VERSION_STRING_LEN_MAX, 0);
	ret = pmc_coProcessUploadSDO();
	if(ret)
		return ret;

	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x1009, 0, moto1_hardwareVersion, VERSION_STRING_LEN_MAX, 0);
	ret = pmc_coProcessUploadSDO();
	if(ret)
		return ret;

	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x100a, 0, moto1_softwareVersion, VERSION_STRING_LEN_MAX, 0);
	ret = pmc_coProcessUploadSDO();
	if(ret)
		return ret;

	ESP_LOGI("pmc", "devName:%s, hardware version:%s, software version:%s.", moto1_devName, moto1_hardwareVersion, moto1_softwareVersion);
	return ret;
}

int8_t pmc_sdoSetAddAcceleration(uint8_t _acc)
{
	int8_t ret;
	uint8_t acc = _acc;

	ESP_LOGI("pmc", "set add acceleration %d.", acc);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6008, 0, (uint8_t *)&acc, sizeof(acc), 0);
	ret = pmc_coProcessDownloadSDO();
	return ret;
}

int8_t pmc_sdoSetDecAcceleration(uint8_t _acc)
{
	int8_t ret;
	uint8_t acc = _acc;

	ESP_LOGI("pmc", "set add acceleration %d.", acc);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6009, 0, (uint8_t *)&acc, sizeof(acc), 0);
	ret = pmc_coProcessDownloadSDO();
	return ret;
}

int8_t pmc_sdoReadCurrentPostion(int32_t *postion)
{
	int8_t ret;
	int32_t currentPostion = 0;

	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x600c, 0, (uint8_t *)&currentPostion, sizeof(currentPostion), 0);
	ret = pmc_coProcessUploadSDO();
	if (ret == 0)
	{
		*postion = currentPostion;
	}
	
	return ret;
}

int8_t pmc_sdoReadCurrentVelocity(int32_t *velocity)
{
	int8_t ret;
	int32_t currentVelocity = 0;

	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x6030, 0, (uint8_t *)&currentVelocity, 4, 0);
	if (ret)
	{
		ESP_LOGE("pmc", "currentVelocity read error1 %d.", ret);	
	}
	ret = pmc_coProcessUploadSDO();
	if (ret == 0)
	{
		if(velocity != NULL)
			*velocity = currentVelocity;
		ESP_LOGD("pmc", "currentVelocity: %d.", currentVelocity);
	}else
	{
		ESP_LOGE("pmc", "currentVelocity read error2 %d.", ret);	
	}
	
	return ret;
}


int8_t pmc_sdoReadSubdivided(uint16_t *subdivied)
{
	int8_t ret;
	uint16_t divided;

	ret = CO_SDOclientUploadInitiate(CO->SDOclient[0], 0x600a, 0, (uint8_t *)&divided, 4, 0);
	ret = pmc_coProcessUploadSDO();
	if (ret == 0)
	{
		*subdivied = divided;
	}
	
	return ret;
}

int8_t pmc_sdoSetSubdivided(uint16_t subdivied)
{
	int8_t ret;
	if(subdivied != 0 && subdivied != 2 && subdivied != 4 && subdivied != 8 && subdivied != 16 && 
	   subdivied != 32 && subdivied != 64 && subdivied != 128 && subdivied != 256)
	{
		ESP_LOGE("pmc", "set moto subdivided parameter error.");
		return -1;
	}
	   
	ESP_LOGI("pmc", "set moto subdivided %d.", subdivied);
	ret = CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x6008, 0, (uint8_t *)&subdivied, sizeof(subdivied), 0);
	ret = pmc_coProcessDownloadSDO();
	return ret;
}

int8_t pmc_adoPvModeSetVelocity(int32_t velocity, uint32_t _acceleration, uint32_t timeout)
{
	uint16_t u16_cmd ;
	uint32_t acceleration = _acceleration;
 	int32_t i32_currentVelocity = 0;
	// int32_t i32_lastVelocity = 0;
 	int32_t i32_targetVelocity = velocity;

	// set operator mode
	pmc_sdoSetOperatorMode(PMC_PROFILE_VELOCITY_MODE);

	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x602d, 1, (uint8_t *)&acceleration, sizeof(acceleration), 0); // 加速度, pps/s
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x602d, 2, (uint8_t *)&acceleration, sizeof(acceleration), 0); // 减速度, pps/s
	
	// set target velocity
	ESP_LOGI("pmc", "set velocity %d.", i32_targetVelocity);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x602e, 3, (uint8_t *)&i32_targetVelocity, sizeof(i32_targetVelocity), 0);

	// start
	u16_cmd = (1<<4);
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x602e, 1, (uint8_t *)&u16_cmd, sizeof(u16_cmd), 0);
	pmc_coProcessDownloadSDO();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	timeout -= 100;

	pmc_sdoReadCurrentVelocity(&i32_currentVelocity);
	while (!TARGET_IS_DONE(i32_targetVelocity, i32_currentVelocity))
	{
		if(0 == pmc_sdoReadCurrentVelocity(&i32_currentVelocity))
		{
			ESP_LOGI("pmc", "pv velocity:%d-%d, acceleration:%d.", i32_targetVelocity, i32_currentVelocity, acceleration);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if(timeout > 10)
			timeout -= 10;
		else
			return -1;
	}

	return 0;
}

void pmc_sdoMotorTest(void)
{
	int8_t ret = 0;
	int32_t currentPostion = 0;
	int32_t targetPostion = 0;
	int32_t i32_currentVelocity = 0;
	uint16_t subdivided = 0;
	int32_t steps = 40000; // 4000 steps per circle，编码器分辨率的4倍

	pmc_sdoReadDeviceInfomation();
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	pmc_sdoReadCurrentPostion(&currentPostion);
	pmc_sdoReadSubdivided(&subdivided);
	pmc_sdoReadCurrentVelocity(&i32_currentVelocity);
	ESP_LOGI("pmc", "moto subdivied:%d.", subdivided);
	// ESP_LOGI("pmc", "moto i32_currentVelocity:%d.", i32_currentVelocity);

	subdivided = 64;
	pmc_sdoSetSubdivided(subdivided);

	// clear error status
	ESP_LOGI("pmc", "clear error status.");
	pmc_sdoClearError();

	// motor set to a+
	pmc_sdoSetAddAcceleration(4); //
	// motor set to a-
	pmc_sdoSetDecAcceleration(4);

	// motor set to postion mode
	ESP_LOGI("pmc", "1. motor set to postion mode.");
	pmc_sdoSetOperatorMode(PMC_POSTION_MODE);

	// motor set to max speed
	ESP_LOGI("pmc", "motor set speed.");
	pmc_sdoSetStartVelocity(600);
	pmc_sdoSetStopVelocity(600);
	pmc_sdoSetMaxVelocity(80000); // 200 circles/min

	ESP_LOGI("pmc", "1.1 motor set fordward direction.");
	pmc_sdoSetDirection(PMC_FORWARD_DIC);

	ESP_LOGI("pmc", "1.1.1 motor set target postion1.");
	pmc_sdoMoveSteps(steps);
	targetPostion = currentPostion + steps;
	while (!TARGET_IS_DONE(targetPostion, currentPostion))
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		pmc_sdoReadCurrentPostion(&currentPostion);
		ESP_LOGI("pmc", "postion %d:%d.", targetPostion,currentPostion);
	}

	ESP_LOGI("pmc", "1.1.2 motor set target postion2.");
	pmc_sdoMoveSteps(steps);
	targetPostion = currentPostion + steps;
	while (!TARGET_IS_DONE(targetPostion, currentPostion))
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		pmc_sdoReadCurrentPostion(&currentPostion);
		ESP_LOGI("pmc", "postion %d:%d.", targetPostion,currentPostion);
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ESP_LOGI("pmc", "1.2 motor set backward direction.");
	pmc_sdoSetDirection(PMC_BACKWARD_DIC);

	ESP_LOGI("pmc", "1.2.1 motor set target postion3.");
	pmc_sdoMoveSteps(steps);
	targetPostion = currentPostion - steps;
	while (!TARGET_IS_DONE(targetPostion, currentPostion))
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		pmc_sdoReadCurrentPostion(&currentPostion);
		ESP_LOGI("pmc", "postion %d:%d.", targetPostion,currentPostion);
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// motor set to velocity mode
	ESP_LOGI("pmc", "2. motor set to profile velocity mode.");
	pmc_adoPvModeSetVelocity(8000, 1000, 20000);
	pmc_adoPvModeSetVelocity(9000, 1000, 20000);
	pmc_adoPvModeSetVelocity(0000, 1000, 20000);

	uint16_t u16_parameter = 0;
	CO_SDOclientDownloadInitiate(CO->SDOclient[0], 0x602e, 1, (uint8_t *)&u16_parameter, sizeof(u16_parameter), 0); // set halt = 1
	ESP_LOGI("pmc", "motor profile velocity test done.");
	vTaskDelay(5000 / portTICK_PERIOD_MS);
}

// endfile

#ifndef PMC_H_
#define PMC_H_

#include <stdint.h>
#include "CANopen.h"
#include "CO_OD.h"

#define MAX_MOTORS 2 //Maximum number of motors

#define PMC_POSTION_MODE             0
#define PMC_VELOCITY_MODE             1
#define PMC_PVT_MODE                2    // 多轴插补，在同一时间坐标上对多个轴的位置、速度进行精确控制
#define PMC_PROFILE_POSTION_MODE     4
#define PMC_PROFILE_VELOCITY_MODE     5

#define PMC_FORWARD_DIC        1 // 正方向
#define PMC_BACKWARD_DIC    0 // 反方向 

/*Constants*/
static const uint32_t STAT_Enabled = (1 << 0);
static const uint32_t STAT_Error = (1 << 1);
static const uint32_t STAT_Warning = (1 << 2);
static const uint32_t STAT_Moving = (1 << 3);
static const uint32_t STAT_Reached = (1 << 4);
static const uint32_t STAT_Limit = (1 << 5);
static const uint32_t STAT_FollowingError = (1 << 6);
static const uint32_t STAT_HomingDone = (1 << 7);
static const uint32_t STAT_Toggle = (1 << 8);
static const uint32_t STAT_CommandToggle = (1 << 9);
static const uint32_t STAT_CommandError = (1 << 10);
static const uint32_t STAT_StopOrHalt = (1 << 11);
static const uint32_t STAT_LimitCurrent = (1 << 12);
static const uint32_t STAT_LimitVel = (1 << 13);
static const uint32_t STAT_LimitPos = (1 << 14);
static const uint32_t STAT_LimitPWM = (1 << 15);
static const uint32_t STAT_LimitSetpointVq = (1 << 16);
static const uint32_t STAT_LimitSetpointVd = (1 << 17);
static const uint32_t STAT_ComOperational = (1 << 18);
static const uint32_t STAT_ComStarted = (1 << 19);
static const uint32_t STAT_OverTemperature = (1 << 20);
static const uint32_t STAT_MotorOvervoltage = (1 << 21);
static const uint32_t STAT_MotorUndervoltage = (1 << 22);
static const uint32_t STAT_Blockage = (1 << 23);
static const uint32_t STAT_ParamCmdExec = (1 << 24);
static const uint32_t STAT_BallastCircuit = (1 << 25);
static const uint32_t STAT_Direction = (1 << 26);
static const uint32_t STAT_Overload = (1 << 27);


static const uint8_t ERROR_STATUS_OverTempratureShutdown     = (1<<0);
static const uint8_t ERROR_STATUS_CoilErrorA                = (1<<1);
static const uint8_t ERROR_STATUS_CoilErrorB                = (1<<2);
static const uint8_t ERROR_STATUS_OverCurrentA                = (1<<3);
static const uint8_t ERROR_STATUS_OverCurrentB                = (1<<4);
static const uint8_t ERROR_STATUS_LowVoltageFault            = (1<<5);

static const uint8_t DRIVER_STATUS_Ext1Stop                    = (1<<0);
static const uint8_t DRIVER_STATUS_Ext2Stop                    = (1<<1);
static const uint8_t DRIVER_STATUS_Locked                    = (1<<2);
static const uint8_t DRIVER_STATUS_Busy                        = (1<<3);
static const uint8_t DRIVER_STATUS_Ext3Stop                    = (1<<4);
static const uint8_t DRIVER_STATUS_PvtFifoEmpty                = (1<<5);
static const uint8_t DRIVER_STATUS_PvtFifoDownLimit            = (1<<6);
static const uint8_t DRIVER_STATUS_PvtFifoUpLimit            = (1<<7);

static const uint8_t CMD_NOP = 0x0;
static const uint8_t CMD_ClearError = 0x1;
static const uint8_t CMD_QuickStop = 0x2;
static const uint8_t CMD_Halt = 0x3;
static const uint8_t CMD_Continue = 0x4;
static const uint8_t CMD_Update = 0x5;

static const int16_t OPERATION_MODE = PMC_VELOCITY_MODE;

/**
 * @brief Struct for all nessesary registers in the object-dictionary
 *
 */
typedef struct motorRegister_s
{
    uint8_t     *softwareVersion;        // 0x100a
    uint8_t     *hardwareVersion;        // 0x1009
    uint8_t     *errorStatus;            // 0x6000，对应错误位写1清除
    uint8_t     *driverStatus;            // 0x6001，除busy位，均可写1清除
    uint8_t     *direction;                // 0x6002
    uint8_t     *operationMode;            // 0x6005                
    uint8_t     *stop;                    // 0x6020, 0-正常，1-停止
    uint8_t     *enableRd;                // 0x600e, 0-失能，1-使能
    uint8_t     *enableWr;                // 0x600e, 0-失能，1-使能

    uint32_t     *maxSpeed;                // 0x6003, maxSpeed
    uint32_t     *offsetPostion;            // 0x6004，busy状态下命令将被忽略
    uint16_t     *startSpeed;            // 0x6006
    uint16_t     *stopSpeed;                // 0x6007
    uint8_t        *addAcceleration;        // 0x6008. 加速度，0~8档
    uint8_t        *devAcceleration;        // 0x6009, 减速度，0~8档
    int32_t     *absolutePosition;        // 0x601c
    int32_t     *currentPostion;        // 0x600c, 当前位置
    int32_t     *currentSpeed;            // 0x6030, 当前速度，闭环，>0 正方向，<0 负方向            
} motorRegister;

/**
 * @brief Construct a new Pmc object
 *
 */
void pmc_moto(void);

/**
 * @brief Initialize motor by setting the RPDO- and TPDO-Mapping
 *
 * @param CO Pointer to CANopen object
 * @param nodeId CANopen node ID
 * @param tpdoNum Relevant TPDO-Number (Count from 0)
 * @return int8_t 0 = No Error, -n = Errorcode of mapRPDO + mapTPDO
 */
int8_t pmc_init(CO_t *CO, uint8_t nodeId, uint8_t tpdoNum);

/**
 * @brief Read motor hard version and software version
 *
 * @return int8_t 0 = No Error, -n = Errorcode
 */
int8_t pmc_readVersion(void);

/**
 * @brief Clear potential errors and reenable drive
 *
 * @return int8_t 0 = No Error, -n = Errorcode
 */
int8_t pmc_clearError(void);

/**
 * @brief Execute quick-stop (Quick-Stop-Deceleration). See also: "continueMovement(void)"
 *
 * @return int8_t 0 = No Error, -n = Errorcode
 */
int8_t pmc_quickStop(void);

/**
 * @brief Execute halt (General deceleration). See also: "continueMovement(void)"
 *
 * @return int8_t 0 = No Error, -n = Errorcode
 */
int8_t pmc_halt(void);

/**
 * @brief Continues movement after QuickStop or Halt.
 *
 * @return int8_t 0 = No Error, -n = Errorcode
 */
int8_t pmc_continueMovement(void);

/**
 * @brief Change motor status to "Operation enabled" or "Switch on disabled"
 *
 * @param value 1 = "Operation enabled" ; 0 = "Switch on disabled"
 * @return int8_t 0 = No Error, -n = Value of Motor Error-Register (see Motor-Documentation)
 */
// int8_t pmc_setEnable(uint8_t value);

/**
 * @brief Set the motor speed
 *
 * @param speed Motor speed
 * @return int8_t 0 = No Error, -n = Value of Motor Error-Register (see Motor-Documentation)
 */
int8_t pmc_setSpeed(int32_t speed);

/**
 * @brief Process pending SDO-Download
 *
 * @return int8_t 0 = No Error, -n = Errorcode of "CO_SDOclientDownload(void)"
 */
int8_t pmc_coProcessDownloadSDO(void);

/**
 * @brief Process pending SDO-Upload
 *
 * @return int8_t 0 = No Error, -n = Errorcode of "CO_SDOclientUpload(void)"
 */
int8_t pmc_coProcessUploadSDO(void);

/**
 * @brief Changes the PDO-Mapping of given RPDO
 *
 * @param pdoNumber PDO to change (0-n)
 * @param nodeId Node ID
 * @param mappedObjects Array with objects to be mapped
 * @param numMappedObjects Nummber of mapped objects
 * @return int8_t 0 = No Error, -n = Errorcode of "coProcessDownloadSDO(void)"
 */
int8_t pmc_mapRPDO(uint8_t pdoNumber, uint8_t nodeId, uint32_t *mappedObjects, uint8_t numMappedObjects);

/**
 * @brief Changes the PDO-Mapping of given TPDO
 *
 * @param pdoNumber PDO to change (0-n)
 * @param nodeId Node ID
 * @param mappedObjects Array with objects to be mapped
 * @param numMappedObjects Nummber of mapped objects
 * @param type Transmission Type
 * @param eventTime TPDO event time in 100us steps
 * @param inhibitTime TPDO inhibit time in 100us steps
 * @return int8_t 0 = No Error, -n = Errorcode of "coProcessDownloadSDO(void)"
 */
int8_t pmc_mapTPDO(uint8_t pdoNumber, uint8_t nodeId, uint32_t *mappedObjects, uint8_t numMappedObjects, uint8_t type, uint16_t eventTime, uint16_t inhibitTime);


// void pmc_move_steps(uint32_t steps);
void pmc_sdoMotorTest(void);

#endif /* PMC_H_ */

# ESP32-SKIDS-NANO-CANOPEN

工程拷贝自nathanRamaNoodles/CANopen-ESP32-nodes.git
工程目的：
在ESP32上作为主站控制基于CANOPEN的电机驱动。

## 相关文件介绍

* CO_config.h 配置CAN的端口、缓存、周期、超时等参数;
* CO_driver_target.h 定义了一些基本的参数，大小端、数据结构、宏定义等，在CO_driver.h中被包含;
* CO_driver.h CANOPENNODE不同平台的接口头文件，分别定义了函数CO_CANsetConfigurationMode、CO_CANsetNormalMode、CO_CANmodule_init、CO_CANmodule_disable、CO_CANrxMsg_readIdent、CO_CANrxBufferInit、CO_CANtxBufferInit、CO_CANsend、CO_CANclearPendingSyncPDOs、CO_CANverifyErrors;
* CO_driver.c 接口文件的实现，文件中用到的一些宏定义在ESP-IDF编译环境中提供;
* modul_config.h 外设定义，如电机的站地址、CAN接口波特率等的定义;
* CO_OD.c CO_OD.h 外设的字典文件，由工具[CANopenEditor](https://github.com/CANopenNode/CANopenEditor)自动生成;

## CANOPEN启动过程

* mainTask函数中直接调用CO_init尝试初始化，入口参数会传递当前设备ID、当前设备波特率
* CO_init函数位于CANOpen.c中，先调用CO_new完成对CANopen协议栈对象CO的创建和初始化，CO在CANOpen.c中定为为空指针，对象实体为COO,同样在CANOpen.c中被定义。
* 然后，CO_init函数调用CO_CANinit，进一步调用CO_CANmodule_init，此函数中完成对结构体CANmodulePointer，timingConfig，filterConfig，generalConfig的初始化配置。然后函数逐层返回。
* 返回CO_init后继续执行，调用CO_CANopenInit完成CANOPENnODE协议栈的初始化，根据配置依次调用CO_SDO_init，CO_EM_init，CO_NMT_init，CO_CANtxBufferInit，CO_LSSmaster_init，CO_SYNC_init，CO_TIME_init，CO_RPDO_init，CO_TPDO_init，CO_HBconsumer_init，CO_SDOclient_init，CO_trace_init。这里的配置主要在CO_OD.h中完成，此文件由CANopenEditor工具自动生成。
* 返回mainTask函数后继续执行，创建定时器，定时器会调用coMainTask函数，此函数中依次调用CO_process_SYNC，CO_process_RPDO，CO_process_TPDO函数，SYNC先判断是否在同步时间窗内，如果在完成RPDO和TPDO数据的同步。
* mainTask函数继续执行，调用CO_CANsetNormalMode，进一步调用can_driver_install、can_start、定义定时调用CO_CANinterrupt函数(函数执行can接口的数据缓存)。至此完成了协议栈和平台CAN端口的启动工作。
* CO_sendNMTcommand发送NMT命令，启动电机
* dunker_init完成电机初始化。
* 完成初始化后程序进入while循环，循环中100ms为周期调用CO_process函数处理，完成对NMT、EM、SDO、HBconsumer报文的处理。
* 进入死循环后还有2个定时器任务在执行分别是1s执行一次的coMainTask和CO_CANinterrupt。
* CO_CANinterrupt中接收到的报文会通过buffer->pFunct上的回调函数CO_XX_receive对报文信息进行处理。这些回调函数是在初始化时调用CO_CANrxBufferInit函数时注册的，CO_XX_init函数中都会调用这个注册函数。这些回调函数会完成对收到报文的解析，并调用SET_CANrxNew发送数据到达消息。最终大SDO、EM、NMT、HB消息会在CO_process函数中通过子函数调用被处理，处理后会调用CLEAR_CANrxNew清理新消息标志。
* 其中CO_TPDO_process和其中CO_RPDO_process会由coMainTask中的CO_process_TPDO/CO_process_RPDO函数调用，也会由CO_CANinterrupt的回调函数调用。



// clang-format off
/*******************************************************************************
    CANopen Object Dictionary definition for CANopenNode v1 to v2

    This file was automatically generated by CANopenEditor v4.0-104-g6f50f73

    https://github.com/CANopenNode/CANopenNode
    https://github.com/CANopenNode/CANopenEditor

    DON'T EDIT THIS FILE MANUALLY !!!!
*******************************************************************************/
// For CANopenNode V2 users, C macro `CO_VERSION_MAJOR=2` has to be added to project options
#ifndef CO_VERSION_MAJOR
 #include "CO_driver.h"
 #include "CO_OD.h"
 #include "CO_SDO.h"
#elif CO_VERSION_MAJOR < 4
 #include "301/CO_driver.h"
 #include "CO_OD.h"
 #include "301/CO_SDOserver.h"
#else
 #error This Object dictionary is not compatible with CANopenNode v4.0 and up!
#endif

/*******************************************************************************
   DEFINITION AND INITIALIZATION OF OBJECT DICTIONARY VARIABLES
*******************************************************************************/


/***** Definition for RAM variables *******************************************/
struct sCO_OD_RAM CO_OD_RAM = {
           CO_OD_FIRST_LAST_WORD,

/*1000*/ 0x0000L,
/*1001*/ 0x0L,
/*1003*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1005*/ 0x40000080L,
/*1006*/ 0x186A0L,
/*1008*/ {'0'},
/*1009*/ {'0'},
/*100A*/ {'0'},
/*1010*/ {0x0001L, 0x0001L, 0x0001L, 0x0001L},
/*1011*/ {0x0001L, 0x0001L, 0x0001L, 0x0001L},
/*1017*/ 0x3E8,
/*1018*/ {0x4L, 0x50555349L, 0x0000L, 0x0000L, 0x0000L},
/*1200*/ {{0x2L, 0x0600L, 0x0580L}},
/*1280*/ {{0x3L, 0x0605L, 0x0585L, 0x5L}},
/*1400*/ {{0x6L, 0x0185L, 0xFEL},
/*1401*/ {0x6L, 0x0285L, 0xFEL},
/*1402*/ {0x6L, 0x80000385L, 0xFEL},
/*1403*/ {0x6L, 0x80000485L, 0xFEL},
/*1404*/ {0x6L, 0x80000180L, 0xFEL}},
/*1600*/ {{0x3L, 0x60000008L, 0x60010008L, 0x600C0020L},
/*1601*/ {0x2L, 0x60300020L, 0x600C0020L},
/*1602*/ {0x0L},
/*1603*/ {0x0L},
/*1604*/ {0x8L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L}},
/*1800*/ {{0x6L, 0x0205L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L},
/*1801*/ {0x6L, 0x0305L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L},
/*1802*/ {0x6L, 0x0405L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L},
/*1803*/ {0x6L, 0x0505L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L},
/*1804*/ {0x6L, 0x80000180L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L}},
/*1A00*/ {{0x0L},
/*1A01*/ {0x3L, 0x60030020L, 0x60010008L, 0x60050008L},
/*1A02*/ {0x2L, 0x601C0020L, 0x600C0020L},
/*1A03*/ {0x1L, 0x60200008L},
/*1A04*/ {0x8L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L, 0x20008L}},
/*1F80*/ 0x0001L,
/*1F89*/ 0x0000L,
/*2000*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
/*6000*/ 0x0L,
/*6001*/ 0x0L,
/*6002*/ 0x0L,
/*6003*/ 0x0000L,
/*6004*/ 0x0000L,
/*6005*/ 0x0L,
/*6006*/ 0x00,
/*6007*/ 0x00,
/*6008*/ 0x0L,
/*6009*/ 0x0L,
/*600A*/ 0x00,
/*600B*/ 0x258,
/*600C*/ 0x0000L,
/*600D*/ 0x0L,
/*600E*/ 0x0L,
/*600F*/ {0x2L, 0x2L, 0x0L},
/*6011*/ {0x2L, 0x00, 0x0000L},
/*6012*/ 0x00,
/*6013*/ {0x2L, 0x0L, 0x0L},
/*6015*/ 0x0002L,
/*6017*/ 0x00,
/*6018*/ {0x2L, 0x0L, 0x0L},
/*6019*/ {0x2L, 0x0L, 0x0000L, 0x0L},
/*601A*/ 0x64,
/*601B*/ 0x0L,
/*601C*/ 0x0000L,
/*6020*/ 0x1L,
/*6021*/ 0x1F4,
/*6026*/ 0x8,
/*6027*/ 0x8L,
/*6028*/ 0x40,
/*6029*/ 0x0L,
/*602A*/ 0x0L,
/*602B*/ 0x00,
/*602C*/ {0x3L, 0x0L, 0x0000L, 0x0000L},
/*602D*/ {0x4L, 0x7D00L, 0x7D00L, 0x0258L, 0x0258L},
/*602E*/ {0x4L, 0x00, 0x00, 0x7D00L, 0x0000L},
/*602F*/ {0x6L, 0x0L, 0x00, 0x64, 0x1E, 0x0000L, 0xFA00L},
/*6030*/ 0x0000L,

           CO_OD_FIRST_LAST_WORD,
};


/***** Definition for ROM variables *******************************************/
struct sCO_OD_ROM CO_OD_ROM = {
           CO_OD_FIRST_LAST_WORD,


           CO_OD_FIRST_LAST_WORD,
};


/***** Definition for EEPROM variables *******************************************/
struct sCO_OD_EEPROM CO_OD_EEPROM = {
           CO_OD_FIRST_LAST_WORD,


           CO_OD_FIRST_LAST_WORD,
};


/***** Definition for PERSIST_COMM variables *******************************************/
struct sCO_OD_PERSIST_COMM CO_OD_PERSIST_COMM = {
           CO_OD_FIRST_LAST_WORD,

/*1007*/ 0xF4240L,
/*1014*/ 0x0080L,
/*1015*/ 0x00,
/*1016*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1019*/ 0x0L,
/*1029*/ {0x0L, 0x0L},

           CO_OD_FIRST_LAST_WORD,
};




/*******************************************************************************
   STRUCTURES FOR RECORD TYPE OBJECTS
*******************************************************************************/


/*0x1018*/ const CO_OD_entryRecord_t OD_record1018[5] = {
           {(void*)&CO_OD_RAM.identity.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.identity.vendorID, 0x86, 0x4 },
           {(void*)&CO_OD_RAM.identity.productCode, 0x86, 0x4 },
           {(void*)&CO_OD_RAM.identity.revisionNumber, 0x86, 0x4 },
           {(void*)&CO_OD_RAM.identity.serialNumber, 0x86, 0x4 },
};

/*0x1200*/ const CO_OD_entryRecord_t OD_record1200[3] = {
           {(void*)&CO_OD_RAM.SDOServerParameter[0].numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.SDOServerParameter[0].COB_IDClientToServer, 0x86, 0x4 },
           {(void*)&CO_OD_RAM.SDOServerParameter[0].COB_IDServerToClient, 0x86, 0x4 },
};

/*0x1280*/ const CO_OD_entryRecord_t OD_record1280[4] = {
           {(void*)&CO_OD_RAM.SDOClientParameter[0].numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.SDOClientParameter[0].COB_IDClientToServer, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.SDOClientParameter[0].COB_IDServerToClient, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.SDOClientParameter[0].nodeIDOfTheSDO_Server, 0x0E, 0x1 },
};

/*0x1400*/ const CO_OD_entryRecord_t OD_record1400[3] = {
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[0].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[0].COB_IDUsedByRPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[0].transmissionType, 0x0E, 0x1 },
};

/*0x1401*/ const CO_OD_entryRecord_t OD_record1401[3] = {
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[1].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[1].COB_IDUsedByRPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[1].transmissionType, 0x0E, 0x1 },
};

/*0x1402*/ const CO_OD_entryRecord_t OD_record1402[3] = {
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[2].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[2].COB_IDUsedByRPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[2].transmissionType, 0x0E, 0x1 },
};

/*0x1403*/ const CO_OD_entryRecord_t OD_record1403[3] = {
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[3].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[3].COB_IDUsedByRPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[3].transmissionType, 0x0E, 0x1 },
};

/*0x1404*/ const CO_OD_entryRecord_t OD_record1404[3] = {
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[4].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[4].COB_IDUsedByRPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOCommunicationParameter[4].transmissionType, 0x0E, 0x1 },
};

/*0x1600*/ const CO_OD_entryRecord_t OD_record1600[4] = {
           {(void*)&CO_OD_RAM.RPDOMappingParameter[0].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject1, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject2, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject3, 0x8E, 0x4 },
};

/*0x1601*/ const CO_OD_entryRecord_t OD_record1601[3] = {
           {(void*)&CO_OD_RAM.RPDOMappingParameter[1].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject1, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject2, 0x8E, 0x4 },
};

/*0x1602*/ const CO_OD_entryRecord_t OD_record1602[1] = {
           {(void*)&CO_OD_RAM.RPDOMappingParameter[2].numberOfMappedObjects, 0x0E, 0x1 },
};

/*0x1603*/ const CO_OD_entryRecord_t OD_record1603[1] = {
           {(void*)&CO_OD_RAM.RPDOMappingParameter[3].numberOfMappedObjects, 0x0E, 0x1 },
};

/*0x1604*/ const CO_OD_entryRecord_t OD_record1604[9] = {
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject1, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject2, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject3, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject4, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject5, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject6, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject7, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject8, 0x8E, 0x4 },
};

/*0x1800*/ const CO_OD_entryRecord_t OD_record1800[7] = {
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].COB_IDUsedByTPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].transmissionType, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].inhibitTime, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].compatibilityEntry, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].eventTimer, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[0].SYNCStartValue, 0x0E, 0x1 },
};

/*0x1801*/ const CO_OD_entryRecord_t OD_record1801[7] = {
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].COB_IDUsedByTPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].transmissionType, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].inhibitTime, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].compatibilityEntry, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].eventTimer, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[1].SYNCStartValue, 0x0E, 0x1 },
};

/*0x1802*/ const CO_OD_entryRecord_t OD_record1802[7] = {
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].COB_IDUsedByTPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].transmissionType, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].inhibitTime, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].compatibilityEntry, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].eventTimer, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[2].SYNCStartValue, 0x0E, 0x1 },
};

/*0x1803*/ const CO_OD_entryRecord_t OD_record1803[7] = {
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].COB_IDUsedByTPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].transmissionType, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].inhibitTime, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].compatibilityEntry, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].eventTimer, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[3].SYNCStartValue, 0x0E, 0x1 },
};

/*0x1804*/ const CO_OD_entryRecord_t OD_record1804[7] = {
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].COB_IDUsedByTPDO, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].transmissionType, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].inhibitTime, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].compatibilityEntry, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].eventTimer, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.TPDOCommunicationParameter[4].SYNCStartValue, 0x0E, 0x1 },
};

/*0x1A00*/ const CO_OD_entryRecord_t OD_record1A00[1] = {
           {(void*)&CO_OD_RAM.TPDOMappingParameter[0].numberOfMappedObjects, 0x0E, 0x1 },
};

/*0x1A01*/ const CO_OD_entryRecord_t OD_record1A01[4] = {
           {(void*)&CO_OD_RAM.TPDOMappingParameter[1].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject1, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject2, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject3, 0x8E, 0x4 },
};

/*0x1A02*/ const CO_OD_entryRecord_t OD_record1A02[3] = {
           {(void*)&CO_OD_RAM.TPDOMappingParameter[2].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject1, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject2, 0x8E, 0x4 },
};

/*0x1A03*/ const CO_OD_entryRecord_t OD_record1A03[2] = {
           {(void*)&CO_OD_RAM.TPDOMappingParameter[3].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject1, 0x8E, 0x4 },
};

/*0x1A04*/ const CO_OD_entryRecord_t OD_record1A04[9] = {
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].numberOfMappedObjects, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject1, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject2, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject3, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject4, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject5, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject6, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject7, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject8, 0x8E, 0x4 },
};

/*0x600F*/ const CO_OD_entryRecord_t OD_record600F[3] = {
           {(void*)&CO_OD_RAM.EXTStopParameter.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.EXTStopParameter.externalStopEnable, 0x3E, 0x1 },
           {(void*)&CO_OD_RAM.EXTStopParameter.externalStopConfig, 0x3E, 0x1 },
};

/*0x6011*/ const CO_OD_entryRecord_t OD_record6011[3] = {
           {(void*)&CO_OD_RAM.GPIOParameter.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.GPIOParameter.GPIODirection, 0x8E, 0x2 },
           {(void*)&CO_OD_RAM.GPIOParameter.GPIOConfig, 0x8E, 0x4 },
};

/*0x6013*/ const CO_OD_entryRecord_t OD_record6013[3] = {
           {(void*)&CO_OD_RAM.OCPParameter.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.OCPParameter.ocpth, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.OCPParameter.ocpdeg, 0x0E, 0x1 },
};

/*0x6018*/ const CO_OD_entryRecord_t OD_record6018[3] = {
           {(void*)&CO_OD_RAM.offlineParameter1.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.offlineParameter1.commandNumber, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.offlineParameter1.offlineAutoRun, 0x0E, 0x1 },
};

/*0x6019*/ const CO_OD_entryRecord_t OD_record6019[4] = {
           {(void*)&CO_OD_RAM.offlineParameter2.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.offlineParameter2.commandPointer, 0x0E, 0x1 },
           {(void*)&CO_OD_RAM.offlineParameter2.command, 0x8E, 0x4 },
           {(void*)&CO_OD_RAM.offlineParameter2.saveOfflineCommand, 0x0E, 0x1 },
};

/*0x602C*/ const CO_OD_entryRecord_t OD_record602C[4] = {
           {(void*)&CO_OD_RAM.stepPositionNotification.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.stepPositionNotification.notificationStatus, 0x3E, 0x1 },
           {(void*)&CO_OD_RAM.stepPositionNotification.stepNotificationPosition1, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.stepPositionNotification.stepNotificationPosition2, 0xBE, 0x4 },
};

/*0x602D*/ const CO_OD_entryRecord_t OD_record602D[5] = {
           {(void*)&CO_OD_RAM.PP_PVModeParameter1.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter1.PP_PVModeAcceleration, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter1.PP_PVModeDeceleration, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter1.PP_PVModeStartSpeed, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter1.PP_PVModeStopSpeed, 0xBE, 0x4 },
};

/*0x602E*/ const CO_OD_entryRecord_t OD_record602E[5] = {
           {(void*)&CO_OD_RAM.PP_PVModeParameter2.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter2.PP_PVModeControlWord, 0xBE, 0x2 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter2.PP_PVModeStatusWord, 0xBE, 0x2 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter2.PP_PVModeRunSpeed, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.PP_PVModeParameter2.PP_PVModeTargetPosition, 0xBE, 0x4 },
};

/*0x602F*/ const CO_OD_entryRecord_t OD_record602F[7] = {
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.numberOfEntries, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.analogPositioningEnable, 0x3E, 0x1 },
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.analogStartAD_Code, 0xBE, 0x2 },
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.intervalTimeOfAnalogPositioningAdjustment, 0xBE, 0x2 },
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.analogTriggerValue, 0xBE, 0x2 },
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.analogPositioningMinimumPosition, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.analogPositioningControlParameters.analogPositioningMaximumPosition, 0xBE, 0x4 },
};

/*******************************************************************************
   OBJECT DICTIONARY
*******************************************************************************/
const CO_OD_entry_t CO_OD[CO_OD_NoOfElements] = {
{0x1000, 0x00, 0x86,  4, (void*)&CO_OD_RAM.deviceType},
{0x1001, 0x00, 0x06,  1, (void*)&CO_OD_RAM.errorRegister},
{0x1003, 0x10, 0x8E,  4, (void*)&CO_OD_RAM.preDefinedErrorField[0]},
{0x1005, 0x00, 0x8E,  4, (void*)&CO_OD_RAM.COB_ID_SYNCMessage},
{0x1006, 0x00, 0x8E,  4, (void*)&CO_OD_RAM.communicationCyclePeriod},
{0x1007, 0x00, 0x8F,  4, (void*)&CO_OD_PERSIST_COMM.synchronousWindowLength},
{0x1008, 0x00, 0x06,  1, (void*)&CO_OD_RAM.manufacturerDeviceName},
{0x1009, 0x00, 0x06,  1, (void*)&CO_OD_RAM.manufacturerHardwareVersion},
{0x100A, 0x00, 0x06,  1, (void*)&CO_OD_RAM.manufacturerSoftwareVersion},
{0x1010, 0x04, 0x8E,  4, (void*)&CO_OD_RAM.storeParameters[0]},
{0x1011, 0x04, 0x8E,  4, (void*)&CO_OD_RAM.restoreDefaultParameters[0]},
{0x1014, 0x00, 0x8F,  4, (void*)&CO_OD_PERSIST_COMM.COB_ID_EMCY},
{0x1015, 0x00, 0x8F,  2, (void*)&CO_OD_PERSIST_COMM.inhibitTimeEMCY},
{0x1016, 0x08, 0x8F,  4, (void*)&CO_OD_PERSIST_COMM.consumerHeartbeatTime[0]},
{0x1017, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.producerHeartbeatTime},
{0x1018, 0x04, 0x00,  0, (void*)&OD_record1018},
{0x1019, 0x00, 0x0F,  1, (void*)&CO_OD_PERSIST_COMM.synchronousCounterOverflowValue},
{0x1029, 0x02, 0x0F,  1, (void*)&CO_OD_PERSIST_COMM.errorBehavior[0]},
{0x1200, 0x02, 0x00,  0, (void*)&OD_record1200},
{0x1280, 0x03, 0x00,  0, (void*)&OD_record1280},
{0x1400, 0x02, 0x00,  0, (void*)&OD_record1400},
{0x1401, 0x02, 0x00,  0, (void*)&OD_record1401},
{0x1402, 0x02, 0x00,  0, (void*)&OD_record1402},
{0x1403, 0x02, 0x00,  0, (void*)&OD_record1403},
{0x1404, 0x02, 0x00,  0, (void*)&OD_record1404},
{0x1600, 0x03, 0x00,  0, (void*)&OD_record1600},
{0x1601, 0x02, 0x00,  0, (void*)&OD_record1601},
{0x1602, 0x00, 0x00,  0, (void*)&OD_record1602},
{0x1603, 0x00, 0x00,  0, (void*)&OD_record1603},
{0x1604, 0x08, 0x00,  0, (void*)&OD_record1604},
{0x1800, 0x06, 0x00,  0, (void*)&OD_record1800},
{0x1801, 0x06, 0x00,  0, (void*)&OD_record1801},
{0x1802, 0x06, 0x00,  0, (void*)&OD_record1802},
{0x1803, 0x06, 0x00,  0, (void*)&OD_record1803},
{0x1804, 0x06, 0x00,  0, (void*)&OD_record1804},
{0x1A00, 0x00, 0x00,  0, (void*)&OD_record1A00},
{0x1A01, 0x03, 0x00,  0, (void*)&OD_record1A01},
{0x1A02, 0x02, 0x00,  0, (void*)&OD_record1A02},
{0x1A03, 0x01, 0x00,  0, (void*)&OD_record1A03},
{0x1A04, 0x08, 0x00,  0, (void*)&OD_record1A04},
{0x1F80, 0x00, 0x8E,  4, (void*)&CO_OD_RAM.NMTStartup},
{0x1F89, 0x00, 0x8E,  4, (void*)&CO_OD_RAM.bootTime},
{0x2000, 0x0A, 0x0E,  1, (void*)&CO_OD_RAM.errorStatusBits[0]},
{0x6000, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.motorStatus},
{0x6001, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.controlStatus},
{0x6002, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.motorDirection},
{0x6003, 0x00, 0xBE,  4, (void*)&CO_OD_RAM.maxSpeed},
{0x6004, 0x00, 0xBE,  4, (void*)&CO_OD_RAM.stepRelativePosition},
{0x6005, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.workMode},
{0x6006, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.startSpeed},
{0x6007, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.endSpeed},
{0x6008, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.accValue},
{0x6009, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.decValue},
{0x600A, 0x00, 0xBE,  2, (void*)&CO_OD_RAM.microStep},
{0x600B, 0x00, 0xBE,  2, (void*)&CO_OD_RAM.peakCurrent},
{0x600C, 0x00, 0xBE,  4, (void*)&CO_OD_RAM.motorPosition},
{0x600D, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.motorEnableRx},
{0x600E, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.motorEnableTx},
{0x600F, 0x02, 0x00,  0, (void*)&OD_record600F},
{0x6011, 0x02, 0x00,  0, (void*)&OD_record6011},
{0x6012, 0x00, 0xBE,  2, (void*)&CO_OD_RAM.GPIOValue},
{0x6013, 0x02, 0x00,  0, (void*)&OD_record6013},
{0x6015, 0x00, 0x8E,  4, (void*)&CO_OD_RAM.motionTunning},
{0x6017, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.stallParameter},
{0x6018, 0x02, 0x00,  0, (void*)&OD_record6018},
{0x6019, 0x03, 0x00,  0, (void*)&OD_record6019},
{0x601A, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.extStopDebounceDelay},
{0x601B, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.stallConfig},
{0x601C, 0x00, 0xBE,  4, (void*)&CO_OD_RAM.stepAbsolutePosition},
{0x6020, 0x00, 0x3E,  1, (void*)&CO_OD_RAM.abortStep},
{0x6021, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.linesPerRev},
{0x6026, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.thePositionEaBetweenPulseCountAndRealCount},
{0x6027, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.adjustValueUsedToRecoverSpeedFromStall},
{0x6028, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.stallLength},
{0x6029, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.torqueLoopEnable},
{0x602A, 0x00, 0x0E,  1, (void*)&CO_OD_RAM.powerOffSavePositionEnable},
{0x602B, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.analogInputAdCode},
{0x602C, 0x03, 0x00,  0, (void*)&OD_record602C},
{0x602D, 0x04, 0x00,  0, (void*)&OD_record602D},
{0x602E, 0x04, 0x00,  0, (void*)&OD_record602E},
{0x602F, 0x06, 0x00,  0, (void*)&OD_record602F},
{0x6030, 0x00, 0xBE,  4, (void*)&CO_OD_RAM.realTimeSpeed},
};
// clang-format on

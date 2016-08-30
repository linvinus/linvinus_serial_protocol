
#ifndef _SERIAL_PROTOCOL_MODULES_H_
#define _SERIAL_PROTOCOL_MODULES_H_

#include "serial_protocol.h"

#ifdef CH_KERNEL_VERSION
  #define SD_PROTOCOL_SLAVE
#else
  #define SD_PROTOCOL_MASTER
#endif

#define EAT(...)
#define EXPAND(...) __VA_ARGS__

#define MSG(...) { EXPAND(__VA_ARGS__) },

#ifdef SD_PROTOCOL_MASTER
  #define DEFINE_ON_MASTER(...) EXPAND(__VA_ARGS__)
  #define DEFINE_ON_SLAVE(...)
  #define MASTER_SLAVE_RXTX(...) DEFINE_ON_MASTER(__VA_ARGS__) DEFINE_ON_SLAVE(__VA_ARGS__)
  #define MASTER_ONLY_RX(data_size,data,callback,arg) data_size, data, callback, arg,0,NULL,NULL,NULL
  #define SLAVE_ONLY_RX (data_size,data,callback,arg) 0,NULL,NULL,NULL,data_size,data,NULL,NULL
#else
  #define DEFINE_ON_MASTER(...)
  #define DEFINE_ON_SLAVE(...) EXPAND(__VA_ARGS__)
  #define MASTER_SLAVE_RXTX(...) DEFINE_ON_MASTER(__VA_ARGS__) DEFINE_ON_SLAVE(__VA_ARGS__)
  #define MASTER_ONLY_RX(data_size,data,callback,arg) 0,NULL,NULL,NULL,data_size,data,NULL,NULL
  #define SLAVE_ONLY_RX (data_size,data,callback,arg) data_size,data,callback,arg,0,NULL,NULL,NULL
#endif


/*************** MASTER ONLY PARTS ************************************/
#ifdef SD_PROTOCOL_MASTER

uint8_t sd_printf_callback(uint16_t data_size,uint8_t *data,void *arg);

#endif
/*************** MASTER ONLY END **************************************/




/*************** SLAVE ONLY PARTS **************************************/
#ifdef SD_PROTOCOL_SLAVE

uint8_t on_RobotCFG_update(uint16_t data_size,uint8_t *data,void *arg);
#endif
/*************** SLAVE ONLY END****************************************/




/*************** COMMON ************************************************/



typedef struct {
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
}RobotCFG_t,*RobotCFG_ptr;

RobotCFG_t RobotCFG;
const RobotCFG_ptr const pRobotCFG=&RobotCFG;

typedef struct SerialProtocolCmd_t SerialProtocolCmd_t;


typedef enum {
  _SP_START = SP_SYSTEM_MESSAGE,
  SP_CONFIGURATION,
  SP_PRINTF
}SerialPacketType_t;

SerialProtocolCmd_t SD_CMDS[]={
  /*0 SP_SYSTEM_MESSAGE*/
  MSG(MASTER_SLAVE_RXTX(0, NULL, NULL, NULL, 0, NULL, NULL, NULL))
  /*1 SP_CONFIGURATION*/
  MSG(DEFINE_ON_SLAVE(sizeof(RobotCFG_t), &RobotCFG, on_RobotCFG_update, NULL, sizeof(RobotCFG_t), &RobotCFG, NULL, NULL)
     DEFINE_ON_MASTER(sizeof(RobotCFG_t), &RobotCFG, NULL, NULL, sizeof(RobotCFG_t), &RobotCFG, NULL, NULL)
     )
  /*2 SP_PRINTF*/
  MSG(MASTER_ONLY_RX(0, NULL, sd_printf_callback, NULL))
  };
  //

uint16_t SD_CMDS_COUNT = sizeof(SD_CMDS)/sizeof(SerialProtocolCmd_t);

#endif /* _SERIAL_PROTOCOL_MODULES_H_ */

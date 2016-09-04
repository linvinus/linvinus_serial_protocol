
/*  SPtr config */
#include "SPrt.h"
#include <avr/pgmspace.h>

typedef enum {
  _SP_START = SP_SYSTEM_MESSAGE,
  SP_CONFIGURATION,
  SP_PRINTF
}SerialPacketType_t;

typedef struct {
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
}RobotCFG_t,*RobotCFG_ptr;

RobotCFG_t RobotCFG;
const RobotCFG_ptr pRobotCFG=&RobotCFG;

uint8_t on_RobotCFG_update(uint16_t data_size,uint8_t *data,void *arg);

//PROGMEM is not allowed there, because it also requre special access in serial_protocol.c
const  SerialProtocolCmd_t SD_CMDS[] ={
  {0,NULL,NULL,NULL,0,NULL,NULL,NULL},                              /*SP_SYSTEM_MESSAGE*/
  {sizeof(RobotCFG_t),pRobotCFG,on_RobotCFG_update,NULL,sizeof(RobotCFG_t),pRobotCFG,NULL,NULL},        /*SP_CONFIGURATION*/
  {0,NULL,NULL,NULL,0,NULL,NULL,NULL},                              /*SP_PRINTF*/
  };
  //

const uint16_t SD_CMDS_COUNT = sizeof(SD_CMDS)/sizeof(SerialProtocolCmd_t);

SPrt sprt;
/* END of SPtr*/

unsigned long last_tick;

void setup() {
  // put your setup code here, to run once:
  sprt.begin(115200); //forget about Serial!!! Serial is used for communication.
  sprt.set_print_cmd(SP_PRINTF);
  last_tick = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  sprt.update();
  
  if( (millis() - last_tick) >500 ){
    print_message();//some task every 500ms
    last_tick = millis();  
  }
}

void print_message(){
  //sprt.printf(SP_PRINTF,"new: %d %d %d %d\r\n",(&RobotCFG)->A,(&RobotCFG)->B,(&RobotCFG)->C,(&RobotCFG)->D);
  
  //sprt.receive(SP_CONFIGURATION,true);
  //sprt.write(SP_PRINTF,(uint8_t*)"test\r\n",8);
  
}


uint8_t on_RobotCFG_update(uint16_t data_size,uint8_t *data,void *arg){
  (void) arg;//unused
  (void) data_size;//constant,unused
  RobotCFG_t * cfg = (RobotCFG_t*) data;
  //sprt.printf(SP_PRINTF,"got: %d %d %d %d\r\n",cfg->A,cfg->B,cfg->C,cfg->D);
  cfg->A++;
  cfg->B++;
  cfg->C=cfg->A + cfg->A;
  cfg->D=cfg->C + cfg->B;
  //sprt.printf(SP_PRINTF,"new: %d %d %d %d\r\n",cfg->A,cfg->B,cfg->C,cfg->D);
  return 0;//not used
}




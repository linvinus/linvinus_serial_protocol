# linvinus_serial_protocol

Simple, multi platform binary protocol with low overhead and delivery guarantee.

## Quick description


Simple, multi platform binary protocol with very low overhead and delivery guarantee.

For easy communications between controllers over UART or SPI(not implemented yet)

 __Overhead:__
 * 1 start byte
 * 4 bytes header,
 * 1 additional byte for cobs encoding
 * total: 6bytes

minimal packet size is 6 bytes including 2bytes for user payload data (see fast message section)

Protocol is bidirectionall, host or controller may send any message at any time,

protocol message contain header + body, max body size is limited to 246 bytes.

body may contain any byte

4bits counter, command ID and body size is used for commands identification and synchronization

Body content is protected with checksum, messages with wrong checksum is rejected with error notification message.

GET commands is used to receive data from remote side

SET commands is used to set data on remote side, may be with confirmation or without confirmation

For confirmation or error notification purpose, special "system messages" are used.

To simplify user protocol, special array is used to store all commands (SD_CMDS[]),

this array must be the same on both sides of connection.

## Quick start
for chibios os add C sources to your build scrips
```
LIBSRC += srial_protocol/COBS/cobs.c \
          serial_protocol/chibios/serial_protocol_lld.c \
          serial_protocol/serial_protocol.c

LIBINC += serial_protocol serial_protocol/COBS serial_protocol/chibios

```

create and include into include dir commands file
```
#ifndef _SERIAL_PROTOCOL_MODULES_H_
#define _SERIAL_PROTOCOL_MODULES_H_


#include "serial_protocol.h"

typedef struct {
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
}RobotCFG_t,*RobotCFG_ptr;

RobotCFG_t RobotCFG;
const RobotCFG_ptr const pRobotCFG=&RobotCFG;

typedef struct SerialProtocolCmd_t SerialProtocolCmd_t;

SerialProtocolCmd_t SD_CMDS[]={
  {0,NULL,NULL,NULL,0,NULL,NULL,NULL},                              /*SP_SYSTEM_MESSAGE*/
  {sizeof(RobotCFG_t),&RobotCFG,NULL,NULL,sizeof(RobotCFG_t),&RobotCFG,NULL,NULL},        /*SP_CONFIGURATION*/
  };
  //

uint16_t SD_CMDS_COUNT = sizeof(SD_CMDS)/sizeof(SerialProtocolCmd_t);

#endif /* _SERIAL_PROTOCOL_MODULES_H_ */

```

record `{0,NULL,NULL,NULL,0,NULL,NULL,NULL},` - is default command, must be present without changes.

record `{sizeof(RobotCFG_t),&RobotCFG,NULL,NULL,sizeof(RobotCFG_t),&RobotCFG,NULL,NULL},` is the first command with index `1`

where `RobotCFG_t` is structure witch will be used for `serial_protocol_set_cmd_sync(1)` or `serial_protocol_get_cmd_sync(1,1)`

include this file on host and on controller side.

Now you can fill structure on the host side, then send it to controller with command `serial_protocol_set_cmd_sync(1,1);`

on controller side this structure will automatically updated.

after command `serial_protocol_get_cmd_sync(1)` structure will be filled with data from remote side.

postfix `_sync` means wait for operation complete or timeout (500ms by default)

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */


#ifndef _SERIAL_PROTOCOL_H_
#define _SERIAL_PROTOCOL_H_

#include <stdint.h>

#define SD_BUFFER_LENGTH 255

#define SD_SEQ_ISCONFIRM(S)   ( ((uint8_t)(S)) & ((uint8_t)0x80)       )
#define SD_SEQ_MASK(S)        ( ((uint8_t)(S)) & ((uint8_t)0b1111)     )
#define SD_SEQ_SYSMES_MASK(S) ( ((uint8_t)(S)) & ((uint8_t)0b01110000) )
#define SD_SEQ_CREATE(seq, confirm) (confirm? ( ((uint8_t)SD_SEQ_MASK(seq)) | ((uint8_t)0x80)) : (((uint8_t)SD_SEQ_MASK(seq)) & ((uint8_t)~0x80)) )

#define SD_CMD_ISGET(cmd)      ( ((uint8_t)(cmd)) & ((uint8_t)0x80)  )
#define SD_CMD_INDEX_MASK(cmd) ( ((uint8_t)(cmd)) & ((uint8_t)~0x80) )
#define SD_CMD_CREATE_GET(cmd) ( ((uint8_t)(cmd)) | ((uint8_t)0x80)  )
#define SD_CMD_CREATE_SET(cmd) ( ((uint8_t)(cmd)) & ((uint8_t)~0x80) )


#include "serial_protocol_lld.h"

typedef uint8_t (*SD_CALLBACK)(uint16_t data_size,uint8_t *data,void *arg);

typedef struct SerialProtocolCmd_t SerialProtocolCmd_t;

struct SerialProtocolCmd_t{
  uint16_t rx_data_size;
  void *rx_data;
  SD_CALLBACK rx_callback;
  void *rx_arg;
  uint16_t tx_data_size;
  void *tx_data;
  SD_CALLBACK tx_callback;
  void *tx_arg;
};


extern SerialProtocolCmd_t SD_CMDS[];
extern uint16_t SD_CMDS_COUNT;


typedef enum {//mask  0b01110000
  SP_OK             = 0b00000000,
  SP_UNKNOWNCMD     = 0b00010000,
  SP_WRONGCHECKSUMM = 0b00100000,
  SP_WRONGSIZE      = 0b00110000,
  SP_VERSION        = 0b01000000
}SerialPacketSystemMessageReason_t;

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

  void serial_protocol_main_loop_iterate(void);
  int serial_protocol_get_cmd_async(uint8_t cmd);
  int32_t serial_protocol_get_cmd_sync(uint8_t cmd);
  int32_t serial_protocol_set_cmd_sync(uint8_t cmd, uint8_t confirm);
  uint8_t calculate_version_checksumm(void);
  int32_t serial_protocol_get_cmds_version(void);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _SERIAL_PROTOCOL_H_ */

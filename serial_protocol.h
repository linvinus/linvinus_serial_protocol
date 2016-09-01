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

#define SD_RET_OK                   (0)
#define SD_RET_ERR1                 (SD_RET_OK-1) /* trivial error */
#define SD_RET_LOCK_ERR             (SD_RET_OK-2) /* LOCK error */
#define SD_RET_PROTOCOL_ERR         (SD_RET_OK-3) /* PROTOCOL error */
#define SD_RET_TIME_ERR             (SD_RET_OK-4) /* TIME error */

#define CONVERT_CHKSUMM(summ) ( ((uint8_t)summ) != 0xFF ? (uint8_t)~(summ) : 0xFF )

#define SD_DEFAULT_TIMEOUT 500 /*in ms*/
#define SD_SYNC 1
#define SD_ASYNC 0

#include "serial_protocol_lld.h"

/* Quick description
 *
 * Simple, multi platform binary protocol with very low overhead and delivery guarantee.
 * For easy communications between controllers over UART or SPI(not implemented yet)
 *
 * Overhead:
 *  1 start byte
 *  4 bytes header,
 *  1 additional byte for cobs encoding
 *  total: 6bytes
 *
 * minimal packet size is 6 bytes including 2bytes for user payload data (see fast message section)
 *
 * Protocol is bidirectionall, host or controller may send any message in any time,
 * protocol message contain header + body, max body size is limited to 246 bytes.
 * body may contain any byte
 *
 * 4bits counter, command ID and body size is used for commands identification and synchronization
 * Body content is protected with checksum, messages with wrong checksum is rejected with error notification message.
 * GET commands is used to receive data from remote side
 * SET commands is used to set data on remote side, may be with confirmation or without confirmation
 * For confirmation or error notification purpose, special "system messages" are used.
 *
 * To simplify user protocol, special array is used to store all commands (SD_CMDS[]),
 * this array must be the same on both sides of connection.
 *
 * */

/* Packet format */

/* REGULAR MESSAGE [0]cobsencoded([sequence][cmd][size][invchksumm][body0][body1][body2]...[bodyNsize])
 * [0] - COBS_SYMBOL
 * header 4 bytes
 * body >=0 bytes
 * */
typedef struct{
 uint8_t sequence;  // 0-3bits = sequence,4-6bits - not used, 7th bit (0x80) indicate that comfirm  is requested
 uint8_t cmd;       // 0-6bits = cmd, 7th bit indicate GET (0x80) or SET command
 uint8_t size;      // body size
 uint8_t invchksumm;// body inverse check summ
}sd_header_t, *psd_header_t;

/*  SYSTEM MESSAGE FORMAT [0]cobsencoded([sequence][cmd][size][invchksumm])
 *  [0] - COBS_SYMBOL
 *  system messages 4 bytes, never have a body
 *
 * typedef struct{
 *  uint8_t sequence;  // 0-3bits = sequence,4-6bits - system message reason
 *  uint8_t cmd;       // always == SP_SYSTEM_MESSAGE
 *  uint8_t size;      // cmd for which this system mssage generated, without 7th bit
 *  uint8_t invchksumm;// status in system message (any arbitrary value)
 * }sd_header_t, *psd_header_t;
 *
 * system message reason description and invchksumm meaning
 *  SP_OK - delivery confirmation,
 *          invchksumm for set CMD only, contain uint8_t rxcallback answer if cmd supports callback
 *
 *  SP_UNKNOWNCMD - unknown cmd
 *          invchksumm contain body size
 *
 *  SP_WRONGCHECKSUMM - wrong checksumm
 *          invchksumm contain original invchksumm
 *
 *  SP_WRONGSIZE - wrong body size
 *          invchksumm contain original body size
 *
 *  SP_VERSION - report checksum
 *          invchksumm contain checksum of SD_CMDS array
 * */



typedef uint8_t (*SD_FAST_MESSAGE_CALLBACK_t)(sd_header_t *hdr);
typedef void (*SD_PROTOCOL_INFORM_CALLBACK_t)(uint8_t sequence,uint8_t cmd,uint8_t state);

typedef uint8_t (*SD_CALLBACK)(uint16_t data_size,uint8_t *data,void *arg);

typedef struct SerialProtocolCmd_t SerialProtocolCmd_t;

struct SerialProtocolCmd_t{
  uint16_t rx_data_size; /*16 bit better for memory align, used only as uint8_t*/
  void *rx_data;
  SD_CALLBACK rx_callback;
  void *rx_arg;
  uint16_t tx_data_size; /*16 bit better for memory align, used only as uint8_t*/
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

#define SP_SYSTEM_MESSAGE  0

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  /*standard user functions*/
  int32_t sprt_receive(uint8_t cmd, uint8_t confirm);
  int32_t sprt_send(uint8_t cmd, uint8_t confirm);
  int32_t sprt_exchange(uint8_t cmd, uint8_t confirm);

  /*functions for experts*/
  int32_t _sprt_send_with_data(uint8_t cmd, uint8_t *body, uint16_t body_size, uint8_t confirm);
  int32_t _sprt_exchange_with_data(uint8_t cmd, uint8_t *data, uint16_t data_size, uint8_t confirm);
  int32_t _sprt_fast_message(uint8_t raw_cmd, uint8_t dataA, uint8_t dataB, uint8_t confirm);

  /*special user functions*/
  uint8_t sprt_calculate_version_checksumm(void);
  int32_t sprt_receive_cmds_version(void);
  int32_t sprt_printf(uint8_t cmd,const char *fmt,...);
     void sprt_register_fast_message_func(SD_FAST_MESSAGE_CALLBACK_t fn);
     void sprt_register_protocol_inform_func(SD_PROTOCOL_INFORM_CALLBACK_t fn);
  int32_t sprt_wait_system_message(uint8_t sequence, uint8_t cmd, uint32_t timeout_ms);

   /*
    * Declaration of this function is system depenned, see header for your lld system driver
    * 
    * sprt_thread_init(...)
    * 
    * */

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _SERIAL_PROTOCOL_H_ */

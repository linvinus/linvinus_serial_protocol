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


#ifndef _SERIAL_PROTOCOL_LLD_H_
#define _SERIAL_PROTOCOL_LLD_H_

extern uint8_t cobs_buf1[SD_BUFFER_LENGTH];

#define sd_syslock()
#define sd_sysunlock()
#define sd_wait_for_chars(s)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  int serial_protocol_thread_init(char *portname,int portspeed);
  int sd_put_timeout(char b,int time_ms);
  int sd_get_timeout(int time_ms);
  int sd_write_timeout(uint8_t *buff,int size,int time_ms);
  int32_t sd_wait_system_message(uint8_t sequence, uint8_t cmd);
  void sd_protocol_inform(uint8_t sequence,uint8_t cmd,uint8_t state);
  void sd_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state,uint32_t timeout_ms);
  uint16_t sd_lock_buffer(uint32_t time_ms);
  uint16_t sd_unlock_buffer(void);
  void sd_protocol_flush(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /*_SERIAL_PROTOCOL_LLD_H_*/

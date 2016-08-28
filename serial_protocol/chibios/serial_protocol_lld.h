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


#include "ch.h"
#include "hal.h"

extern uint8_t cobs_buf1[SD_BUFFER_LENGTH];

/* sd_get_timeout(ms)
 * return >= 0 - value
 * return <  0 - error
 * */
#define sd_get_timeout(time_ms) sdGetTimeout(&SD1,MS2ST(time_ms))

/*
 * return 0 - SUCCSESS
 * */
#define sd_put_timeout(byte,time_ms) sdPutTimeout(&SD1,byte,MS2ST(time_ms))
/*
 * return - size of written data
 * */
#define sd_write_timeout(buff,size,time_ms) sdWriteTimeout(&SD1,buff,size,MS2ST(time_ms))

#define sd_syslock() chSysLock()
#define sd_sysunlock() chSysUnlock()

#define sd_wait_for_chars(count) \
      while(1){ \
        chSysLock(); \
        if(chIQGetFullI(&((&SD1)->iqueue)) > count) break; \
        chSysUnlock(); \
        chThdYield();/*as fast as possible*/ \
      } \
      chSysUnlock();\

//not used
#define sd_protocol_inform(sequence,cmd,state)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

  void serial_protocol_thread_init(void);
  inline int32_t sd_wait_system_message(uint8_t sequence, uint8_t cmd);
  inline void sd_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state,uint32_t timeout_ms);
  inline uint16_t sd_lock_buffer(uint32_t time_ms);
  inline uint16_t sd_unlock_buffer(void);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /*_SERIAL_PROTOCOL_LLD_H_*/

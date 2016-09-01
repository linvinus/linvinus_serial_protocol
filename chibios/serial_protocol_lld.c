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


#include "ch.h"
#include "hal.h"
#include "serial_protocol.h"
#include "memstreams.h"
#include <stdarg.h> /*va_list for sd_lld_sprintf*/
#include "chprintf.h" /*chvprintf*/

static threads_queue_t sd_protocol_q_waiting;
static BSEMAPHORE_DECL(SD_BUFF_SEM,FALSE);

uint8_t cobs_buf1[SD_BUFFER_LENGTH];

inline int32_t sprt_wait_system_message(uint8_t sequence, uint8_t cmd, uint32_t timeout_ms){
  uint32_t start = chVTGetSystemTimeX();
  uint32_t elapsed = 0;
  do{
    msg_t msg = chThdEnqueueTimeoutS(&sd_protocol_q_waiting,MS2ST(timeout_ms) - elapsed);
    if(msg == MSG_TIMEOUT){
      return SD_RET_TIME_ERR;//timeout
    }else if( SD_SEQ_MASK(msg >> 16) == SD_SEQ_MASK(sequence) &&
              SD_CMD_INDEX_MASK(msg >> 8) == SD_CMD_INDEX_MASK(cmd) ){
          //int32_t system_message = msg ;//& (0x7F << 16 | 0x7F <<8 | 0xFF);//it's oksd_wait_system_message, because usfulldata only in 0x7f7fff
          return (msg & 0xFFFFFF);//Delivered successful
    }
  }while( (elapsed = chVTTimeElapsedSinceX(start)) < MS2ST(timeout_ms) );//osalOsIsTimeWithinX ?
  return SD_RET_TIME_ERR;//timeout anyway
}

inline void sd_lld_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state,uint32_t timeout_ms){
  (void)timeout_ms;
  chThdDequeueAllI(&sd_protocol_q_waiting,((uint32_t)sequence<<16 |(uint32_t)cmd<<8 |state));
}

inline uint16_t sd_lld_lock_buffer(uint32_t time_ms){
  return (chBSemWaitTimeout(&SD_BUFF_SEM,MS2ST(time_ms)) == MSG_OK);
}

inline uint16_t sd_lld_unlock_buffer(){
  chBSemSignal(&SD_BUFF_SEM);
  return 1;//always true
}


static THD_WORKING_AREA(waThreadSerialProtocol, 256);
static THD_FUNCTION(ThreadSerialProtocol, arg) {
  (void)arg;
  while(1){
    //~ _sd_main_loop_iterate();
    _sprt_main_loop_iterate();
    chThdYield();
    //~ chThdSleepMilliseconds(1);
  }//while 1
}//ThreadSerialProtocol


void sprt_thread_init(void){
  chThdQueueObjectInit(&sd_protocol_q_waiting);
  chThdCreateStatic(waThreadSerialProtocol, sizeof(waThreadSerialProtocol), NORMALPRIO+1, ThreadSerialProtocol, NULL);
}

int sd_lld_sprintf(uint8_t *str, size_t size, const char *fmt,va_list ap){
  /* Require :
   * $(CHIBIOS)/os/hal/lib/streams/memstreams.c
   * $(CHIBIOS)/os/hal/lib/streams/chprintf.c
   * */
  //~ va_list ap;
  MemoryStream ms;
  BaseSequentialStream *chp;
  size_t size_wo_nul;
  int retval;

  if (size > 0)
    size_wo_nul = size - 1;
  else
    size_wo_nul = 0;

  /* Memory stream object to be used as a string writer, reserving one
     byte for the final zero.*/
  msObjectInit(&ms, (uint8_t *)str, size_wo_nul, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *)(void *)&ms;
  //~ va_start(ap, fmt);
  retval = chvprintf(chp, fmt, ap);
  //~ va_end(ap);

  /* Terminate with a zero, unless size==0.*/
  if (ms.eos < size)
      str[ms.eos] = 0;

  /* Return number of bytes that would have been written.*/
  return retval;
}

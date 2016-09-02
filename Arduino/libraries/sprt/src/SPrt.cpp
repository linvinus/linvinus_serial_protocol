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

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SPrt.h"
#include "serial_protocol.h"

uint8_t cobs_buf1[SD_BUFFER_LENGTH];

int sprt_lld_put_timeout(char b,int time_ms){
  unsigned long start;
  
  if(sptr_serial->availableForWrite()>0){
    sptr_serial->write(b);
    return 0;//OK
  }else{
    start = millis();
  
    while (sptr_serial->availableForWrite() == 0 && (millis() - start) < time_ms){
       delay(1);//wait 1ms
    }
    
    if(sptr_serial->availableForWrite()>0){
      sptr_serial->write(b);
      return 0;
    }else
      return -1;//ERROR
  }
}

int sprt_lld_get_timeout(int time_ms){
  unsigned long start;

  
  if(sptr_serial->available()>0){
    return sptr_serial->read();//OK
  }else{
    start = millis();
  
    while (sptr_serial->available() == 0 && (millis() - start) < time_ms){
       delay(1);//wait 1ms
    }
    
    if(sptr_serial->available()>0)
      return sptr_serial->read();
    else
      return -1;//ERROR
  }
}

int sprt_lld_write_timeout(uint8_t *buff,int size,int time_ms){
  unsigned long start;
  int sz = size;

  start = millis();
    
  while(sz >0){
    if(sptr_serial->availableForWrite()>0){
      sptr_serial->write(*(buff++));
    }else if(millis() - start > time_ms){
      return -1;//error timeout
    }
  }
  return size;
}

uint8_t last_sys_message[4];

void sprt_lld_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state,uint32_t timeout_ms){
  last_sys_message[0]=state;
  last_sys_message[1]=cmd;
  last_sys_message[2]=sequence;
}

int32_t sprt_wait_system_message(uint8_t sequence, uint8_t cmd, uint32_t timeout_ms){
  unsigned long start = millis();
  
  while(1){
    if(last_sys_message[2] == sequence
    && last_sys_message[1] == cmd){
      return *((int32_t*)last_sys_message);
    }else if(millis() - start > timeout_ms){
      return -1;//error timeout
    }
  }
}

int sprt_lld_sprintf(uint8_t *str, size_t size, const char *fmt,va_list ap){
  int retval = vsnprintf((char *)str,size,fmt,ap); /* Return number of bytes that would have been written.*/
  return retval;
}


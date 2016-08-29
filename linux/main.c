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


/*
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
*/
#include <stdlib.h>
#include <stdio.h>
//~ #include <stdint.h>
#include <termios.h> /*B115200*/
#include <pthread.h> /*for clock_gettime CLOCK_MONOTONIC*/
#include <unistd.h> /*usleep*/

#include "serial_protocol_modules.h"

uint8_t sd_printf_callback(uint16_t data_size,uint8_t *data,void *arg){
  //arguments: hdr->size, cobs_buf_p, NULL
  write(1, data, data_size);//print to std out
  return 0;//not used
}

void sd_protocol_inform_callback(uint8_t sequence,uint8_t cmd,uint8_t state){
  char *c=NULL;
  switch( SD_SEQ_SYSMES_MASK(sequence) ){
    case   SP_OK:
      //~ c="SP_OK";
    break;
    case SP_UNKNOWNCMD:
      c="SP_UNKNOWNCMD";
    break;
    case SP_WRONGCHECKSUMM:
      c="SP_WRONGCHECKSUMM";
    case SP_WRONGSIZE:
      c="SP_WRONGSIZE";
    break;
    case SP_VERSION:
      c="SP_VERSION";
    break;
    default:
      c="Unknown state!";
  }
  
  if(c != NULL)
    fprintf (stderr,"\r\ngot inform cmd(%d)[%d]=%d %s\r\n",(uint8_t)SD_CMD_INDEX_MASK(cmd),(uint8_t)SD_SEQ_MASK(sequence),(uint8_t)state,c);
}

int main(void) {

  setbuf(stdout, NULL);// disable buffering entirely

  if(serial_protocol_thread_init("/dev/rfcomm1",B115200)){
    printf("serial_protocol_thread_init error\r\n");
    exit(EXIT_FAILURE);
  }

  sd_register_protocol_inform_func(sd_protocol_inform_callback);

  

  //check version on first connect
  int32_t remote_checksumm;
  remote_checksumm = serial_protocol_get_cmds_version();


  while(remote_checksumm < 0){
    printf("Checking version...\r\n");
    usleep (1000 * 1000);//1s
    sd_protocol_flush();
    remote_checksumm = serial_protocol_get_cmds_version();
  }

/*
  if( remote_checksumm >= 0){
    int32_t local_checksumm = calculate_version_checksumm();
    if( remote_checksumm == local_checksumm){
      printf("Version OK\r\n");
    }else{
      printf("ERROR: Version missmatch! %d != %d\r\n",local_checksumm,remote_checksumm);
      exit(EXIT_FAILURE);
    }
  }
*/
  struct timespec   start,end;

  while(1){
    //~ printf("\r\n###");
    //~ if(inputAvailable())
      //~ serial_protocol_get_cmd(1);

    //~ printf("###\r\n");
    //~ usleep (500 * 1000);
    printf("\r\n@@@");
    clock_gettime(CLOCK_MONOTONIC, &start);
    int res = serial_protocol_set_cmd_sync(1,1);
    clock_gettime(CLOCK_MONOTONIC, &end);

    printf("@@@ %d s=%d %dms \r\n",res,(int)(end.tv_sec - start.tv_sec), (int)(end.tv_nsec - start.tv_nsec)/1000) ;
    usleep (500 * 1000);
    //~ c = getch();
  }

}

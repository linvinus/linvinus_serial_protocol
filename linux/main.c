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


int main(void) {


  if(serial_protocol_thread_init("/dev/rfcomm1",B115200)){
    printf("serial_protocol_thread_init error\r\n");
    exit(EXIT_FAILURE);
  }

  setbuf(stdout, NULL);// disable buffering entirely

  //check version on first connect
  int32_t remote_checksumm;
  remote_checksumm = serial_protocol_get_cmds_version();


  while(remote_checksumm < 0){
    printf("Checking version...\r\n");
    usleep (1000 * 1000);//1s
    sd_protocol_flush();
    remote_checksumm = serial_protocol_get_cmds_version();
  }

  if( remote_checksumm >= 0){
    int32_t local_checksumm = calculate_version_checksumm();
    if( remote_checksumm == local_checksumm){
      printf("Version OK\r\n");
    }else{
      printf("ERROR: Version missmatch! %d != %d\r\n",local_checksumm,remote_checksumm);
      exit(EXIT_FAILURE);
    }
  }

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

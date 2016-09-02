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
  (void)arg;
  write(1, data, data_size);//print to std out
  return 0;//not used
}

void sd_protocol_inform_callback(uint8_t sequence,uint8_t cmd,uint8_t state){
  char *c=NULL;
  switch( SD_SEQ_SYSMES_MASK(sequence) ){
    case   SP_OK:
      //~ c="SP_OK";
      return;
    break;
    case SP_UNKNOWNCMD:
      c="SP_UNKNOWNCMD";
    break;
    case SP_WRONGCHECKSUMM:
      c="SP_WRONGCHECKSUMM";
    break;
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

RobotCFG_t RobotCFG2;

void check_send_receive(){
  int res;
    /*
     * fill RobotCFG struct with data from remote side
     */
    //~ printf("<%d %d %d %d\r\n",pRobotCFG->A,pRobotCFG->B,pRobotCFG->C,pRobotCFG->D);
    res = sprt_receive(SP_CONFIGURATION,SD_SYNC);
    printf("cmd=%d seq=%d\r\n",SP_CONFIGURATION,SD_SEQ_MASK(res>>16));
    if(res <0) printf("error: sprt_get_cmd(1,1)=%d\r\n",res);
    else{
      //~ printf(">%d %d %d %d\r\n",pRobotCFG->A,pRobotCFG->B,pRobotCFG->C,pRobotCFG->D);
      if( (RobotCFG.A - RobotCFG2.A) != 1 ||
          (RobotCFG.B - RobotCFG2.B) != 1  ){
          printf("error: %d!=%d %d!=%d \r\n",RobotCFG.A, RobotCFG2.A,RobotCFG.B, RobotCFG2.B);
        }

      RobotCFG2 = RobotCFG;
      /*
       * Send RobotCFG struct to remote side,
       * on remote side, after receiving, special callback function will be called,
       * witch will update data in RobotCFG,
       * so, after next sprt_receive we will get updated data
       */
      res = sprt_send(SP_CONFIGURATION,SD_SYNC);
      if(res <0) printf("error: sprt_set_cmd(1,1)\r\n");

    }
}//check_send_receive

void check_exchange(){
  int res;
    /*
     * fill RobotCFG struct with data from remote side
     */
    printf("<%d %d %d %d\r\n",pRobotCFG->A,pRobotCFG->B,pRobotCFG->C,pRobotCFG->D);
    res = sprt_exchange(SP_CONFIGURATION,SD_ASYNC);
    printf("cmd=%d seq=%d\r\n",SP_CONFIGURATION,res);
    res = sprt_wait_system_message(SD_SEQ_MASK(res),SP_CONFIGURATION, SD_DEFAULT_TIMEOUT);
    if(res <0) printf("error: sprt_get_cmd(1,1)=%d\r\n",res);
    else{
      printf(">%d %d %d %d\r\n",pRobotCFG->A,pRobotCFG->B,pRobotCFG->C,pRobotCFG->D);
      if( (RobotCFG.A - RobotCFG2.A) != 1 ||
          (RobotCFG.B - RobotCFG2.B) != 1  ){
          printf("error: %d!=%d %d!=%d \r\n",RobotCFG.A, RobotCFG2.A,RobotCFG.B, RobotCFG2.B);
        }

      RobotCFG2 = RobotCFG;
    }
}//check_send_receive

int main(void) {

  setbuf(stdout, NULL);// disable buffering entirely

  sprt_register_protocol_inform_func(sd_protocol_inform_callback);

  //~ if(sprt_thread_init("/dev/rfcomm1",B115200)){
  if(sprt_thread_init("/dev/ttyUSB0",B115200)){
    printf("sprt_thread_init error\r\n");
    exit(EXIT_FAILURE);
  }

  /* Firstly get version
   * checking  connection
   * */
  int remote_checksumm;
  do{
    usleep (1000 * 1000);//1s
    printf("Checking version...\r\n");
    sprt_lld_flush();
    remote_checksumm = sprt_receive_cmds_version();
  }while(remote_checksumm < 0);

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
  struct timespec   start,end,dt;

  while(1){

    clock_gettime(CLOCK_MONOTONIC, &start);
      //~ check_send_receive();
      check_exchange();//should be twice fast as check_send_receive()
      //~ usleep (1000 * 1000);//1s
    clock_gettime(CLOCK_MONOTONIC, &end);

    sprt_lld_timespec_diff(&start,&end,&dt);

    printf("transaction time: %dms \r\n",(int)(dt.tv_nsec/1000000 + dt.tv_sec*1000)) ;

    //~ usleep (500 * 1000);
    //~ c = getch();
  }

}

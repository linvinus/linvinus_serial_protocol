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

void sd_protocol_inform_callback(uint8_t sequence,uint8_t cmd,uint8_t state,uint8_t from_remote){
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
    fprintf (stderr,"\r\n %s inform cmd(%d)[%d]=%d %s\r\n",(from_remote? "got from remote": "send to remote"),(uint8_t)SD_CMD_INDEX_MASK(cmd),(uint8_t)SD_SEQ_MASK(sequence),(uint8_t)state,c);
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
  int res=0,res2=0;
    /*
     * fill RobotCFG struct with data from remote side
     */
    //~ printf("<%d %d %d %d\r\n",pRobotCFG->A,pRobotCFG->B,pRobotCFG->C,pRobotCFG->D);
    struct timespec   start,end;
    clock_gettime(CLOCK_REALTIME, &start);

    printf("<");
    res = sprt_exchange(SP_CONFIGURATION,SD_SYNC);
    //printf("cmd=%d seq=%d\r\n",SP_CONFIGURATION,res);
    //~ res2 = sprt_wait_system_message(SD_SEQ_MASK(res),SP_CONFIGURATION, SD_DEFAULT_TIMEOUT);
    if(res <0){
      printf("error: sprt_get_cmd(1,1)=%d seq=%d cmd=%d\r\n",res,SD_SEQ_MASK(res),SP_CONFIGURATION);
      printf("error: %d!=%d %d!=%d seq=%d cmd=%d\r\n",RobotCFG.A, RobotCFG2.A,RobotCFG.B, RobotCFG2.B,SD_SEQ_MASK(res),SP_CONFIGURATION);
    }else{
      //~ printf(">%d %d %d %d\r\n",pRobotCFG->A,pRobotCFG->B,pRobotCFG->C,pRobotCFG->D);
      clock_gettime(CLOCK_REALTIME, &end);
      sprt_lld_timespec_diff(&start,&end,&end);
      printf("> dt=%dns ",end.tv_nsec);
      if( (RobotCFG.A - RobotCFG2.A) != 1 ||
          (RobotCFG.B - RobotCFG2.B) != 1  ){
          printf("error: %d!=%d %d!=%d seq=%d cmd=%d\r\n",RobotCFG.A, RobotCFG2.A,RobotCFG.B, RobotCFG2.B,SD_SEQ_MASK(res>> 16),SP_CONFIGURATION);
        }

      RobotCFG2 = RobotCFG;
    }
}//check_send_receive

void set_realtime(){
    struct sched_param par={0};
    par.sched_priority=99;
    if((sched_setscheduler(0,SCHED_FIFO,&par)!=0)){
      printf("Not realtime\r\n");
      printf("please create file /etc/security/limits.d/99-realtime.conf\r\n");
      printf("\t@audio   -  rtprio     99\r\n\t@audio   -  memlock    unlimited\r\n");
      printf("then relogin to the system.\r\n\r\n");
    }else
      printf("Realtime\r\n");

    struct timespec   quantum;
    if(sched_rr_get_interval(0,&quantum)!=0)
	printf("sched_rr_get_interval error");
    else
	printf("current scheduler quantum=%dns\r\n",quantum.tv_nsec);
}

int main(void) {

  setbuf(stdout, NULL);// disable buffering entirely

  set_realtime();

  sprt_register_protocol_inform_func(sd_protocol_inform_callback);

  //~ if(sprt_thread_init("/dev/rfcomm1",B115200)){
  if(sprt_thread_init("/dev/ttyAMA0",B921600)){//B921600
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
//    dt.tv_sec=0; dt.tv_nsec=1;
//    nanosleep(&dt,&dt);//real yield
      check_exchange();//should be twice fast as check_send_receive()
    //usleep (1);
    clock_gettime(CLOCK_MONOTONIC, &end);


    sprt_lld_timespec_diff(&start,&end,&dt);

    printf("transaction time: %dus \r\n",(int)(dt.tv_nsec/1000 + dt.tv_sec*1000000)) ;


    //~ c = getch();
  }

}

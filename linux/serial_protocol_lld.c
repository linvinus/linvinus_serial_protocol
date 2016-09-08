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

#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <stdint.h>
#include <pthread.h>


#include "serial_protocol.h"


/**********************************************************************/
uint8_t cobs_buf1[SD_BUFFER_LENGTH];

/**********************************************************************/
static int TTY_fd;//TTY file descriptor
static pthread_mutex_t mutex_buffer = PTHREAD_MUTEX_INITIALIZER; /*used for private access to cobs_buf1*/
static int sysmsg_fd[2];//pipe fd

/************************ Private functions ***************************/

#define DEBUG_PROCOTOL(...) fprintf (stderr, __VA_ARGS__)
//~ #define DEBUG_SERIAL(...) fprintf (stderr, __VA_ARGS__)
#define DEBUG_SERIAL(...)
//~ #define DEBUG_THREAD_CONDITION(...) fprintf (stderr, __VA_ARGS__)
//~ #define DEBUG_THREAD_CONDITION_VAR(...) __VA_ARGS__
#define DEBUG_THREAD_CONDITION_VAR(...)
#define DEBUG_THREAD_CONDITION(...)

# define timeradd(a, b, result) \
  do { \
    (result)->tv_sec = (a)->tv_sec + (b)->tv_sec; \
    (result)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec; \
    if ((result)->tv_nsec >= 1000000000) \
      { \
        ++(result)->tv_sec; \
        (result)->tv_nsec -= 1000000000; \
      } \
  } while (0)


static int tty_set_interface_attribs (int fd, int speed, int parity){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return SD_RET_ERR1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tcflush(fd, TCIFLUSH);
        cfmakeraw(&tty);

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return SD_RET_ERR1;
        }
        return SD_RET_OK;
}

static void tty_set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;//SD_RET_ERR1
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

/*
int inputAvailable() {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  int rv = select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  if(rv ==-1 || rv == 0) return 0;
  else return 1;
  //~ return (FD_ISSET(STDIN_FILENO, &fds));
}*/

static void* _sprt_lld_thread_fn(void *arg){
  (void)arg;
  while(1){
    _sprt_main_loop_iterate();
    //~ usleep (1 * 1000); timeout in sd_read_byte is enough
  }
  return SD_RET_OK;
}


/************************ Public functions ****************************/

/* sd_get_timeout(ms)
 * return >= 0 - value
 * return <  0 - error
 * */
int sprt_lld_get_timeout(int time_ms){
  uint8_t byte=0;

  struct timeval timeout;
  fd_set set;
  int rv;
  FD_ZERO(&set); /* clear the set */
  FD_SET(TTY_fd, &set); /* add our file descriptor to the set */
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000*time_ms;
  rv = select(TTY_fd + 1, &set, NULL, NULL, &timeout);
  if(rv == -1){
    DEBUG_SERIAL("select error\r\n"); /* an error accured */
    return SD_RET_ERR1;
  }else if(rv == 0){
    DEBUG_SERIAL("sd_read_byte timeout \r\n"); /* a timeout occured */
    return SD_RET_ERR1;
  }else{
     /* there was data to read */
    if(read( TTY_fd, &byte, 1 ) >= 0){
      DEBUG_SERIAL("r=0x%02x ",byte);
      return byte;
    }else{
      DEBUG_SERIAL("r=timeout ");
      return SD_RET_ERR1;
    }
  }
}

/* sd_put_timeout
 * return 0 - SUCCSESS
 * */
int sprt_lld_put_timeout(char b,int time_ms){


  DEBUG_SERIAL("*** w=0x%02d ",(uint8_t)b);



  struct timeval timeout;
  fd_set set;
  int rv;
  FD_ZERO(&set); /* clear the set */
  FD_SET(TTY_fd, &set); /* add our file descriptor to the set */
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000*time_ms;
  rv = select(TTY_fd + 1, NULL, &set, NULL, &timeout);
  if(rv == -1){
    DEBUG_SERIAL("select error\r\n"); /* an error accured */
    return SD_RET_ERR1;
  }else if(rv == 0){
    DEBUG_SERIAL("sd_put_timeout timeout \r\n"); /* a timeout occured */
    return SD_RET_ERR1;
  }else{
     /* there is space to write */
     int e = write(TTY_fd, &b, 1 );
     //~ tcflush(TTY_fd, TCSADRAIN);
    return ( e == 1 ?  SD_RET_OK : SD_RET_ERR1);
  }
}

/* sd_write_timeout
 * return - size of written data
 * */
int sprt_lld_write_timeout(uint8_t *buff,int size,int time_ms){
  int i;
  for(i=0;i<size;i++){
    DEBUG_SERIAL("w[%d]=0x%02x ",i,buff[i]);
  }
  struct timeval timeout;
  fd_set set;
  int rv;
  FD_ZERO(&set); /* clear the set */
  FD_SET(TTY_fd, &set); /* add our file descriptor to the set */
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000*time_ms;
  rv = select(TTY_fd + 1, NULL, &set, NULL, &timeout);
  if(rv == -1){
    DEBUG_SERIAL("select error\r\n"); /* an error accured */
    return SD_RET_ERR1;
  }else if(rv == 0){
    DEBUG_SERIAL("sd_write_timeout timeout \r\n"); /* a timeout occured */
    return SD_RET_ERR1;
  }else{

      //return
      i = write(TTY_fd, buff, size );
      //~ tcflush(TTY_fd, TCSADRAIN);
      return i;
  }
}



int sprt_thread_init(char *portname,int portspeed){

  pthread_t serial_protocol_thread;

  int  iret1;

  //~ char *portname = "/dev/ttyUSB0";
  //~ char *portname = "/dev/rfcomm1";
  DEBUG_PROCOTOL("opeinig port %s\r\n",portname);



  TTY_fd = open (portname, O_RDWR | O_NOCTTY |O_NONBLOCK );//O_NONBLOCK
  if (TTY_fd < 0)
  {
          printf ("error %d opening %s: %s", errno, portname, strerror (errno));
          return SD_RET_ERR1;
  }

  tty_set_interface_attribs (TTY_fd, portspeed, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  tty_set_blocking (TTY_fd, 1);                // set no blocking

  //~ usleep (5000 * 1000);//for /dev/rfcomm1
  pipe(sysmsg_fd);

  iret1 = pthread_create( &serial_protocol_thread, NULL, _sprt_lld_thread_fn, (void*) NULL);
  if(iret1){
    fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
    exit(EXIT_FAILURE);
  }
  return SD_RET_OK;//ok
}

static int32_t _sprt_read_pipe_system_message(struct timeval *timeout){
  int32_t word=0;

  fd_set set;
  int rv;
  FD_ZERO(&set); /* clear the set */
  FD_SET(sysmsg_fd[0], &set); /* add our file descriptor to the set */
  rv = select(sysmsg_fd[0] + 1, &set, NULL, NULL, timeout);
  if(rv == -1){
    DEBUG_THREAD_CONDITION("select error\r\n"); /* an error accured */
    return SD_RET_ERR1;
  }else if(rv == 0){
    DEBUG_THREAD_CONDITION("sd_read_byte timeout \r\n"); /* a timeout occured */
    return SD_RET_ERR1;
  }else{
     /* there was data to read */
    if(read( sysmsg_fd[0], &word, 4 ) >= 0){
      DEBUG_THREAD_CONDITION("r=0x%04x ",word);
      return word;
    }else{
      DEBUG_THREAD_CONDITION("r=timeout ");
      return SD_RET_ERR1;
    }
  }
}

int32_t sprt_wait_system_message(uint8_t sequence, uint8_t cmd, uint32_t timeout_ms){
  DEBUG_THREAD_CONDITION("{");
  int32_t system_message;
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000*timeout_ms;

  do{
    DEBUG_THREAD_CONDITION("-");
    system_message = _sprt_read_pipe_system_message(&timeout) & (0xFFFFFF);
    if(  system_message>0){
      if(SD_SEQ_MASK(system_message >> 16) == SD_SEQ_MASK(sequence) &&
         SD_CMD_INDEX_MASK(system_message >> 8) == SD_CMD_INDEX_MASK(cmd) ){

        DEBUG_THREAD_CONDITION_VAR(struct timespec   now);
        DEBUG_THREAD_CONDITION_VAR(clock_gettime(CLOCK_REALTIME, &now));//CLOCK_MONOTONIC is not supported there :(
        DEBUG_THREAD_CONDITION(" [s=%02d c=%d %d] ",SD_SEQ_MASK(system_message >> 16),SD_CMD_INDEX_MASK(system_message >> 8),now.tv_nsec);
        DEBUG_THREAD_CONDITION("}");
        return system_message;//Delivered successful
      }
    }else{
        DEBUG_THREAD_CONDITION("}");
      return SD_RET_TIME_ERR;//timeout, wrong cmd or sequence
    }

    //not found yet
    if(timeout.tv_usec > 0){
      continue;//not timeout yet, wait a bit more
    }else{
        DEBUG_THREAD_CONDITION("}");
      return SD_RET_TIME_ERR;//timeout, wrong cmd or sequence
    }
  }while(1);

    DEBUG_THREAD_CONDITION("}");
  return SD_RET_TIME_ERR;//timeout anyway
}


void sprt_lld_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state,uint32_t timeout_ms){
  DEBUG_THREAD_CONDITION_VAR(struct timespec   now);
  DEBUG_THREAD_CONDITION_VAR(clock_gettime(CLOCK_REALTIME, &now));//CLOCK_MONOTONIC is not supported there :(

  uint32_t word = ((uint32_t)sequence<<16 |(uint32_t)cmd<<8 |state);

  struct timeval timeout;
  fd_set set;
  int rv;
  FD_ZERO(&set); /* clear the set */
  FD_SET(sysmsg_fd[1], &set); /* add our file descriptor to the set */
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000*timeout_ms;
  rv = select(sysmsg_fd[1] + 1, NULL, &set, NULL, &timeout);
  if(rv > 0){
     /* there is space to write */
     write(sysmsg_fd[1], &word, 4 );
     DEBUG_THREAD_CONDITION(" (s=%02d c=%d %d) ",sequence,cmd,now.tv_nsec);
    //~ return ( e == 1 ?  SD_RET_OK : SD_RET_ERR1);
  }
}

uint16_t sprt_lld_lock_buffer(uint32_t time_ms){
  int               rc;
  struct timespec   timeout,now,dt;
  dt.tv_sec = 0;
  dt.tv_nsec = 1000000*time_ms;//ms

  clock_gettime(CLOCK_REALTIME, &now);//CLOCK_MONOTONIC is not supported there :(

  timeradd(&dt,&now,&timeout);

  rc = pthread_mutex_timedlock(&mutex_buffer,&timeout);

  return ( rc == 0);
}


uint16_t sprt_lld_unlock_buffer(void){
  pthread_mutex_unlock(&mutex_buffer);
  return 1;//always true
}

void sprt_lld_flush(void){
  tcflush(TTY_fd, TCIOFLUSH);
}

int sprt_lld_sprintf(uint8_t *str, size_t size, const char *fmt,va_list ap){
  int retval = vsnprintf((char *)str,size,fmt,ap); /* Return number of bytes that would have been written.*/
  return retval;
}

void sprt_lld_timespec_diff(struct timespec *start,struct timespec *stop,struct timespec *result)
{
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    return;
}

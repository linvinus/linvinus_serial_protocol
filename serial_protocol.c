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
 * Author:  Konstantinov Denis
 * email:   linvinus@gmail.com
 * date:    28.08.2016
 * version: 0.5
 *
 */


#include "serial_protocol.h"
#include "cobs.h"

#include <stdarg.h> /*va_list for sd_printf*/

#define COBSBUF_RAW_DATA_OFFCET 4

#define SD_HEADER_SIZE 4
#define SD_MAX_PACKET (SD_BUFFER_LENGTH - 1 - SD_HEADER_SIZE - COBSBUF_RAW_DATA_OFFCET) /*246 bytes*/

#define MIN(A,B) ( (A) > (B) ? (B) : (A) )

static uint8_t last_sequence;//4bits counter
static uint8_t * const cobs_buf_p = (uint8_t *)&cobs_buf1;

SD_FAST_MESSAGE_CALLBACK_t fast_message_fn = NULL;
SD_PROTOCOL_INFORM_CALLBACK_t protocol_inform_fn = NULL;

/*
 * Because packet whole encoded as header + body,
 * but we parse separately, firstly head then body,
 * we need store cobs-decoder state between calls
 * don't requre mutex protection because used only in single thread
 * */
typedef struct{
  int32_t n;    //current processed byte
  int32_t code; //last code
} cobs_state_t;

static cobs_state_t cobs_state_rx; //store state between calls of cobs_receive_decode

#define cobs_rx_reset() {cobs_state_rx.n = 0;}

static int16_t build_and_send_package(sd_header_t *hdr,uint8_t bodysize,uint8_t* body){
  uint16_t pktsize=0,i;
  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;//after cobs_encode data will be starting from [0]
  uint8_t *hdrp=(uint8_t *)hdr;

  if(sprt_lld_lock_buffer(500)){//try to lock with timeout

    if(bodysize > 0){
      if( body != NULL && bodysize < SD_MAX_PACKET ){
        uint8_t *raw_body = raw + SD_HEADER_SIZE;//reserve space for header
        uint8_t raw_body_checksumm = 0;
        //copy body to temporary buffer, calculate checksumm
        sprt_lld_syslock();
          for( i=0; i < bodysize; i++){
            *(raw_body++) = *(body);
            raw_body_checksumm += *(body++);
          }
          //~ memccpy(,,bodysize,sizeof(uint8_t));
        sprt_lld_sysunlock();

        hdr->size = bodysize;//size
        hdr->invchksumm = CONVERT_CHKSUMM(raw_body_checksumm);//invchksumm
      }else{
        sprt_lld_unlock_buffer();
        return SD_RET_PROTOCOL_ERR; //protocol error, bodysize >0 but body == NULL or too big size
      }
    }

    raw[0] = hdrp[0];//sequence
    raw[1] = hdrp[1];//cmd
    raw[2] = hdrp[2];//size, or some usefull data if bodysize==0
    raw[3] = hdrp[3];//invchksumm, or some usefull data if bodysize==0

    pktsize = bodysize + SD_HEADER_SIZE;//body + header

    pktsize = cobs_encode(raw,pktsize,cobs_buf_p);//move to the left

    if( sprt_lld_put_timeout(COBS_SYMBOL,100) == 0 ){//start of packet
      size_t rc = sprt_lld_write_timeout(cobs_buf_p,pktsize,500);//send encoded data
      sprt_lld_unlock_buffer();
      return (rc == pktsize ? (int16_t)rc : SD_RET_PROTOCOL_ERR ); //-1 //protocol error
    }else{
      sprt_lld_unlock_buffer();
      return SD_RET_PROTOCOL_ERR; //protocol error
    }
  }
    else return SD_RET_LOCK_ERR; //LOCK error

}//build_and_send_package

static int16_t call_callback_build_and_send_package(sd_header_t *hdr,SD_CALLBACK callback){
  uint16_t pktsize=0;
  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;//after cobs_encode data will be starting from [0]
  uint8_t *hdrp=(uint8_t *)hdr;

  if( callback != NULL && sprt_lld_lock_buffer(500)){//try to lock with timeout

    uint8_t *raw_body = raw + SD_HEADER_SIZE;//reserve space for header
    uint8_t bodysize = MIN(SD_MAX_PACKET,hdr->size);//max packet size
    sprt_lld_syslock();
    uint8_t invchksumm = callback(0,raw_body,&bodysize);
    if(invchksumm != 0){//filled data succsessfull
      hdr->size = bodysize;//manually fill header
      hdr->invchksumm = CONVERT_CHKSUMM(invchksumm);
    }else{
      sprt_lld_sysunlock();
      sprt_lld_unlock_buffer();
      return SD_RET_PROTOCOL_ERR; //callback didn't return data
    }
    sprt_lld_sysunlock();


    raw[0] = hdrp[0];//sequence
    raw[1] = hdrp[1];//cmd
    raw[2] = hdrp[2];//size
    raw[3] = hdrp[3];//invchksumm

    pktsize = bodysize + SD_HEADER_SIZE;//body + header

    pktsize = cobs_encode(raw,pktsize,cobs_buf_p);//move to the left

    if( sprt_lld_put_timeout(COBS_SYMBOL,100) == 0 ){//start of packet
      size_t rc = sprt_lld_write_timeout(cobs_buf_p,pktsize,500);//send encoded data
      sprt_lld_unlock_buffer();
      return (rc == pktsize ? (int16_t)rc : SD_RET_PROTOCOL_ERR ); //-1 //protocol error
    }else{
      sprt_lld_unlock_buffer();
      return SD_RET_PROTOCOL_ERR; //protocol error
    }
  }
    else return SD_RET_LOCK_ERR; //LOCK error

}//build_and_send_package

static size_t cobs_receive_decode(size_t pktsize, uint8_t* destination,uint8_t invchksumm){

  uint8_t *dst = destination;
  uint8_t *end = dst + pktsize;
  uint8_t chksumm = 0;
  int32_t c;


  while(dst < end)
  {
      if(cobs_state_rx.n == 0){ //load new code only if previous was fully parsed

        cobs_state_rx.code = sprt_lld_get_timeout(100);

        if(cobs_state_rx.code < 0 || cobs_state_rx.code == COBS_SYMBOL /*|| ( (cobs_state_rx.read_index + cobs_state_rx.code) > end && cobs_state_rx.code != 1)*/)
            return SD_RET_PROTOCOL_ERR;//error

        cobs_state_rx.n = cobs_state_rx.code;
      }

      while( cobs_state_rx.n > 1 ){
        if(dst < end){

          c = sprt_lld_get_timeout(100);

          if(c < 0 || c == COBS_SYMBOL)//COBS_SYMBOL is not allowed there!
            return SD_RET_PROTOCOL_ERR;//error

          cobs_state_rx.n--;
          chksumm += *(dst++) = c;
        }else
          goto FINISH;//package was partially received, got requested pktsize
      }


      if(cobs_state_rx.code != 0xFF && dst != end)
      {
          chksumm += *(dst++) = COBS_SYMBOL;
          cobs_state_rx.n--;
      }
  }
  FINISH:

  //warning: comparison of promoted ~unsigned with unsigned https://gcc.gnu.org/bugzilla/show_bug.cgi?id=38341
  if( invchksumm !=0 && CONVERT_CHKSUMM(chksumm) != invchksumm)
    return SD_RET_PROTOCOL_ERR;//error, broken package, checksum mismatch

  return (dst - destination);
}//cobs_receive_decode

static inline size_t cobs_start_receiving_and_decode(size_t pktsize, uint8_t* destination){
  cobs_rx_reset();
  return cobs_receive_decode(pktsize, destination,0);
}


/* skip current packet body because of some error */
static inline void skip(sd_header_t *hdr){
  uint8_t s = hdr->size;
  while( s-- > 0 && sprt_lld_get_timeout(100) >=0)
    ;
}

static inline void system_message_answer(sd_header_t *hdr, SerialPacketSystemMessageReason_t reason, uint8_t status){
  /*USED SYSTEM MESSAGE FORMAT! SEE ABOVE*/
  hdr->sequence = SD_SEQ_CREATE(hdr->sequence,0)| SD_SEQ_SYSMES_MASK(reason);//remove confirm bit, add reason
  hdr->size = hdr->cmd;                          //store current cmd in size
  hdr->cmd = SP_SYSTEM_MESSAGE;
  hdr->invchksumm = status;
  build_and_send_package(hdr, 0, NULL);
}

static inline void system_message_answer_error(sd_header_t *hdr, SerialPacketSystemMessageReason_t reason, uint8_t status){
  if(protocol_inform_fn != NULL)
    protocol_inform_fn(hdr->sequence | SD_SEQ_SYSMES_MASK(reason), hdr->cmd, status);
  system_message_answer(hdr,reason,status);
}

static inline void skip_and_aswer_error(sd_header_t *hdr, SerialPacketSystemMessageReason_t err, uint8_t status){
  skip(hdr);
  system_message_answer_error(hdr,err,status);
}

uint8_t sprt_calculate_version_checksumm(void){
    uint8_t verchksumm,i;
    verchksumm = SD_MAX_PACKET ^ SD_CMDS_COUNT;
    for(i=0;i<SD_CMDS_COUNT;i++){
      verchksumm ^= (SD_CMDS[i].rx_data_size
                 ^   SD_CMDS[i].tx_data_size
                 ^ ( (SD_CMDS[i].rx_data == NULL)<<1
                   | (SD_CMDS[i].rx_callback == NULL)<<2
                   | (SD_CMDS[i].rx_arg == NULL)<<3
                   | (SD_CMDS[i].tx_data == NULL)<<4
                   | (SD_CMDS[i].tx_callback == NULL)<<5
                   | (SD_CMDS[i].tx_arg == NULL)<<6 ) );
    }
    return verchksumm;
}

/* main protocol thread
 * */
void _sprt_main_loop_iterate(void){
    int32_t c = -1;
    c = sprt_lld_get_timeout(100);

    //~ if(c >=0) sprt_lld_put_timeout(c,100);//loopback

    if(c == COBS_SYMBOL){
      /* got packet delimeter,
       * recieve header
       * */
      uint8_t header[SD_HEADER_SIZE]={0};
      sd_header_t *hdr = (sd_header_t *)header;

      //~ sprt_lld_wait_for_chars(3);//wait for count of SD_HEADER_SIZE

      if( cobs_start_receiving_and_decode(SD_HEADER_SIZE,header) == SD_HEADER_SIZE ){
        //got header

        if(hdr->cmd == SP_SYSTEM_MESSAGE){                                                           /* received protocol system message */
            sprt_lld_broadcast_system_message(hdr->sequence, hdr->size, hdr->invchksumm,500);
            if(protocol_inform_fn != NULL)
              protocol_inform_fn(hdr->sequence, hdr->size, hdr->invchksumm);
        }else if(fast_message_fn != NULL && fast_message_fn(hdr)){                                   /* check for fast message */
          if(SD_SEQ_ISCONFIRM(hdr->sequence)){ //comfirm requested
            //send ak
            system_message_answer(hdr,SP_OK,hdr->invchksumm);
          }
        }else if(hdr->cmd == SD_CMD_CREATE_GET(SP_SYSTEM_MESSAGE) ){                                  /* special case,"get version" */
          system_message_answer(hdr,SP_VERSION,sprt_calculate_version_checksumm());
        }
        else                                                                                          /* regular message */
        {
          last_sequence = SD_SEQ_MASK(hdr->sequence);//used for next outgoing message

          if(hdr->size < SD_MAX_PACKET ){//size is ok

            uint8_t cmd_idx = SD_CMD_INDEX_MASK(hdr->cmd);//remove GET bit

            if( cmd_idx < SD_CMDS_COUNT ){                                        /* known CMD */

              if( hdr->invchksumm != 0 ){//there is some usfull body in incoming packet
                /*
                 * firstly receive incoming data
                 *
                 * */
                if( SD_CMDS[cmd_idx].rx_data_size == hdr->size ){

/*CMD SET fixed size format*/

                  //receive body in temporary buffer
                  if(sprt_lld_lock_buffer(500)){
                    if(cobs_receive_decode(hdr->size, cobs_buf_p, hdr->invchksumm) == hdr->size ){

                      //got body
                      uint8_t* rx_data = SD_CMDS[cmd_idx].rx_data;
                      uint8_t *raw = cobs_buf_p;

                      if(rx_data != NULL){
                        //copy body to destination
                        sprt_lld_syslock();
                        uint16_t i;
                        for(i=0; i < hdr->size; i++){
                          *(rx_data++) = *(raw++);
                        }
                        sprt_lld_sysunlock();
                        sprt_lld_unlock_buffer();
                        rx_data = SD_CMDS[cmd_idx].rx_data;//restore pointer
                      }else
                        rx_data = cobs_buf_p;

                      //callback
                      uint8_t callback_status;
                      if(SD_CMDS[cmd_idx].rx_callback != NULL){
                        callback_status = SD_CMDS[cmd_idx].rx_callback(hdr->size,rx_data,SD_CMDS[cmd_idx].rx_arg);
                      }

                      if(!SD_CMD_ISGET(hdr->cmd)){ //GET cmd is processed later
                        sprt_lld_broadcast_system_message(hdr->sequence, hdr->cmd, hdr->size,500);//inform our self for sync GET commands
                        /* for debug only
                        if(protocol_inform_fn != NULL)
                          protocol_inform_fn(hdr->sequence, hdr->cmd, hdr->size);
                        */

                        if(SD_SEQ_ISCONFIRM(hdr->sequence)){ //comfirm requested
                          //send ak
                          system_message_answer(hdr,SP_OK,callback_status);
                        }
                      }
                    }else {//body error
                      sprt_lld_unlock_buffer();
                      system_message_answer_error(hdr,SP_WRONGCHECKSUMM,hdr->invchksumm); //don't skip because already partially or completely received
                      return;//skip GET
                    }
                  }//sprt_lld_lock_buffer
                  //else - must not happens

                }else{

/* CMD SET variable size format, or wrong configuration */

                  //call tx callback to fill variable data
                  if(SD_CMDS[cmd_idx].rx_callback != NULL){
                    //receive body in temporary buffer
                    if(sprt_lld_lock_buffer(500)){
                      if(cobs_receive_decode(hdr->size, cobs_buf_p, hdr->invchksumm) == hdr->size ){
                        //got body
                        uint8_t callback_status = SD_CMDS[cmd_idx].rx_callback(hdr->size, cobs_buf_p, NULL);
                        sprt_lld_unlock_buffer();

                        if(!SD_CMD_ISGET(hdr->cmd)){ //GET cmd is processed later
                          sprt_lld_broadcast_system_message(hdr->sequence, hdr->cmd, hdr->size,500);//inform our self for sync GET commands

                          if(SD_SEQ_ISCONFIRM(hdr->sequence)){ //comfirm requested
                            //send ak
                            system_message_answer(hdr,SP_OK,callback_status);
                          }
                        }

                      }else {//body error
                        sprt_lld_unlock_buffer();
                        system_message_answer_error(hdr,SP_WRONGCHECKSUMM,hdr->invchksumm); //don't skip because already partially or completely received
                        return;//skip GET
                      }
                    }//sprt_lld_lock_buffer
                    //else - must not happens
                  }else{
                    skip_and_aswer_error(hdr,SP_WRONGSIZE,SD_CMDS[cmd_idx].rx_data_size);//report our prefered size
                  }

                }//end RX variable size format

              }else if(!SD_CMD_ISGET(hdr->cmd) && hdr->size !=0){
                /* SET cmd with broken invchksumm*/
                skip_and_aswer_error(hdr,SP_WRONGCHECKSUMM,hdr->invchksumm);
                return;
              }

/*this section is also always processed for every packet, because of exchange command*/
/* CMD type GET */
              if(SD_CMD_ISGET(hdr->cmd))
              {
                /*
                 * send some data if it was requested
                 *
                 * */
                 hdr->cmd = SD_CMD_INDEX_MASK(hdr->cmd);//remove GET bit from header struct,prevent infinity loop

                if(SD_CMDS[cmd_idx].tx_data_size != 0 ){

/*CMD GET fixed size format*/

                  /*
                   * check size,
                   * skip checking if hdr->invchksumm != 0
                   *   this means "exchange packet" type, receiver will check size anyway
                   *
                   */
                  if( hdr->invchksumm !=0 || SD_CMDS[cmd_idx].tx_data_size == hdr->size){
                    build_and_send_package(hdr,SD_CMDS[cmd_idx].tx_data_size,(uint8_t*)SD_CMDS[cmd_idx].tx_data);

                    //tx callback
                    if(SD_CMDS[cmd_idx].tx_callback != NULL){
                      SD_CMDS[cmd_idx].tx_callback(hdr->size,SD_CMDS[cmd_idx].tx_data,SD_CMDS[cmd_idx].tx_arg);
                    }
                  }else{
                    skip_and_aswer_error(hdr,SP_WRONGSIZE,SD_CMDS[cmd_idx].tx_data_size);
                  }

                }else{
/* CMD GET variable size format, or wrong configuration */
                  if(SD_CMDS[cmd_idx].tx_callback != NULL){//call tx callback to fill variable data
                    call_callback_build_and_send_package(hdr,SD_CMDS[cmd_idx].tx_callback);
                  }else{
                    skip_and_aswer_error(hdr,SP_WRONGSIZE,SD_CMDS[cmd_idx].tx_data_size);//report our prefered size
                  }
                }//end variable size format
              }//end CMD type GET

/* DONE */
            }else{
              skip_and_aswer_error(hdr,SP_UNKNOWNCMD,hdr->size);
            }

          }else{
            skip_and_aswer_error(hdr,SP_WRONGSIZE,hdr->size);
          }
        }//nonsystem message

      }//if got header

    }//if COBS_SYMBOL
}//_sd_main_loop

void sprt_register_fast_message_func(SD_FAST_MESSAGE_CALLBACK_t fn){
  fast_message_fn = fn;
}

void sprt_register_protocol_inform_func(SD_PROTOCOL_INFORM_CALLBACK_t fn){
  protocol_inform_fn = fn;
}

static int32_t _sprt_process_command(sd_header_t *hdr,size_t body_size,uint8_t* body, uint8_t confirm){

    if( build_and_send_package(hdr, body_size, body) == (int16_t)(SD_HEADER_SIZE+ body_size +1) ){
        if(confirm)
          return sprt_wait_system_message(SD_SEQ_MASK(hdr->sequence),SD_CMD_INDEX_MASK(hdr->cmd), SD_DEFAULT_TIMEOUT);//return sd_wait_system_message state, < 0 if error
        else
          return SD_SEQ_MASK(hdr->sequence);//OK
    }else
        return SD_RET_PROTOCOL_ERR;//ERROR
}
/*************** protocol commands ************************************/

/* _sprt_exchange_with_data
 *
 * This is unsafe function but allow more freedom for protocol control.
 * Must be used by experts only.
 *
 * If data == NULL, but data_size != 0 then only receive data with requested size
 *
 * Return: >=0 - if success
 *         <0  - on error
 * */
int32_t _sprt_exchange_with_data(uint8_t cmd, uint8_t *body, uint16_t body_size, uint8_t confirm){
  sd_header_t hdr1;
  sd_header_t  *hdr = &hdr1;

  cmd = SD_CMD_INDEX_MASK(cmd);                       //ensure that command index is correct

  if(cmd < SD_CMDS_COUNT )
  {
    hdr->sequence = SD_SEQ_CREATE(++last_sequence,0); //remove confirm bit, the answer is our confirm
    hdr->cmd = SD_CMD_CREATE_GET(cmd);                //GET
    hdr->size = body_size;                            //GET data size, may be overwritten in build_and_send_package
    hdr->invchksumm = 0;                              //invchksumm must be zero if body_size==0, otherwise will be updated in build_and_send_package

    if(body == NULL) body_size = 0;                   //if body == NULL then don't send any data in build_and_send_package, only receive amount of hdr->size

    return _sprt_process_command(hdr,body_size,body,confirm);
  }else
    return SD_RET_ERR1;//ERROR
}

/* _sprt_send_with_data
 *
 * This is unsafe function but allow more freedom for protocol control.
 * Must be used by experts only.
 *
 * Send any data to remote side,
 * user must enshure that sending data size is the same as on remote size! (except for commands with variable length)
 * Return: >=0 - if success
 *         <0  - on error
 * */
int32_t _sprt_send_with_data(uint8_t cmd, uint8_t *body, uint16_t body_size, uint8_t confirm){
  sd_header_t hdr1;
  sd_header_t  *hdr = &hdr1;

  cmd = SD_CMD_INDEX_MASK(cmd);                               //ensure that command index is correct

  if(cmd < SD_CMDS_COUNT && body != NULL  && body_size != 0)
  {
    hdr->sequence = SD_SEQ_CREATE(++last_sequence,confirm);   //set confirm if required
    hdr->cmd = SD_CMD_CREATE_SET(cmd);                        //SET
    hdr->invchksumm = 0;                                      //invchksumm must be zero if body_size==0, otherwise will be updated in build_and_send_package
    return _sprt_process_command(hdr,body_size,body,confirm);
  }else
    return SD_RET_ERR1;//ERROR
}

/* send fast message
 *
 * This is unsafe function but allow more freedom for protocol control.
 * Must be used by experts only.
 *
 * Return: >=0 - if success
 *         <0  - on error
 * */
int32_t _sprt_fast_message(uint8_t raw_cmd, uint8_t dataA, uint8_t dataB, uint8_t confirm){
    sd_header_t hdr1;
    sd_header_t  *hdr = &hdr1;
    hdr->sequence = SD_SEQ_CREATE(++last_sequence,confirm);    //set confirm if required
    hdr->cmd = raw_cmd;                                        //raw cmd send as is, but answer must be masked with SD_CMD_INDEX_MASK()
    hdr->size = dataA;                                         //raw dataA
    hdr->invchksumm = dataB;                                   //raw dataB
    return _sprt_process_command(hdr,0, NULL,confirm);
}

/* Send SD_CMDS[cmd].tx_data to remote side
 *
 * Return: >=0 - if success
 *         <0  - on error
 * */
int32_t sprt_send(uint8_t cmd, uint8_t confirm){
  return _sprt_send_with_data(cmd, SD_CMDS[cmd].tx_data, SD_CMDS[cmd].tx_data_size, confirm);
}

/*
 * Receive data from remote side into SD_CMDS[cmd].rx_data, or as argument in callback function
 *
 * Incoming data will be stored in SD_CMDS[cmd].rx_data if SD_CMDS[cmd].rx_data != NULL
 * or send as arguments for SD_CMDS[cmd].rx_callback if SD_CMDS[cmd].rx_callback != NULL
 *
 * Return: >=0 - if success
 *         <0  - on error
 * */
int32_t sprt_receive(uint8_t cmd, uint8_t confirm){
    return _sprt_exchange_with_data(cmd,NULL,SD_CMDS[cmd].rx_data_size,confirm);
}

/*
 * Send and receive in single transaction
 *
 * Send SD_CMDS[cmd].tx_data and receive into SD_CMDS[cmd].rx_data
 *
 * */
int32_t sprt_exchange(uint8_t cmd, uint8_t confirm){
    return _sprt_exchange_with_data(cmd,SD_CMDS[cmd].tx_data, SD_CMDS[cmd].tx_data_size,confirm);
}


/* print into buffer then send packet as "cmd", without confirmation about delivery
 * cmd index must be the same as on remote side.
 *
 * Example:
 * describe command on remote side as :
 *     {0,NULL,sd_printf_callback,NULL,0,NULL,NULL,NULL},        //SP_PRINTF
 *
 * callback example function on remote side :
 *  uint8_t sd_printf_callback(uint16_t data_size,uint8_t *data,void *arg){
 *         write(stdout, data, data_size);//print to std out
 *  return 0;//not used
 * }
 *
 * */
int32_t sprt_printf(uint8_t cmd,const char *fmt,...){
  uint16_t pktsize=0;
  uint8_t raw_body_checksumm=0,i=0;
  va_list ap;

  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;//after cobs_encode data will be starting from [0]

  if( sprt_lld_lock_buffer(500)){//try to lock with timeout

    uint8_t *raw_body = raw + SD_HEADER_SIZE;
    uint8_t bodysize = SD_MAX_PACKET;//max packet size



    va_start(ap, fmt);
    bodysize = sprt_lld_sprintf(raw_body, bodysize,fmt,ap);
    va_end(ap);

    for( i=0; i < bodysize; i++){
      raw_body_checksumm += *(raw_body++);
    }

    raw[0] = SD_SEQ_CREATE(++last_sequence,0);
    raw[1] = cmd;
    raw[2] = bodysize;//size
    raw[3] = CONVERT_CHKSUMM(raw_body_checksumm);//invchksumm

    pktsize = bodysize + SD_HEADER_SIZE;//body + header

    pktsize = cobs_encode(raw,pktsize,cobs_buf_p);//move to the left

    if( sprt_lld_put_timeout(COBS_SYMBOL,100) == 0 ){//start of packet
      size_t rc = sprt_lld_write_timeout(cobs_buf_p,pktsize,500);//send encoded data
      sprt_lld_unlock_buffer();
      return (rc == pktsize ? (int16_t)rc : SD_RET_PROTOCOL_ERR ); //-1 //protocol error
    }else{
      sprt_lld_unlock_buffer();
      return SD_RET_PROTOCOL_ERR; //protocol error
    }
  }
    else return SD_RET_LOCK_ERR; //LOCK error

}

int32_t sprt_receive_cmds_version(void){
  int32_t version_checksumm = sprt_receive(0,1);//special system message

  if(version_checksumm < 0) return SD_RET_PROTOCOL_ERR;//error, timeout

  return (version_checksumm & 0xFF);
}

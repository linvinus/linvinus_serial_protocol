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

#define COBSBUF_RAW_DATA_OFFCET 4

#define SD_HEADER_SIZE 4
#define SD_MAX_PACKET (SD_BUFFER_LENGTH - 1 - SD_HEADER_SIZE - COBSBUF_RAW_DATA_OFFCET) /*246 bytes*/

#define MIN(A,B) ( (A) > (B) ? (B) : (A) )
static uint8_t last_sequence;//4bits counter

static uint8_t * const cobs_buf_p = (uint8_t *)&cobs_buf1;

/* Quick description
 *
 * Simple, multi platform binary protocol with very low overhead and delivery guarantee.
 * For easy communications between controllers over UART or SPI(not implemented yet)
 *
 * Overhead:
 *  1 start byte
 *  4 bytes header,
 *  1 additional byte for cobs encoding
 *  total: 6bytes
 *
 * minimal packet size is 6 bytes including 2bytes for user payload data (see fast message section)
 *
 * Protocol is bidirectionall, host or controller may send any message in any time,
 * protocol message contain header + body, max body size is limited to 246 bytes.
 * body may contain any byte
 *
 * 4bits counter, command ID and body size is used for commands identification and synchronization
 * Body content is protected with checksum, messages with wrong checksum is rejected with error notification message.
 * GET commands is used to receive data from remote side
 * SET commands is used to set data on remote side, may be with confirmation or without confirmation
 * For confirmation or error notification purpose, special "system messages" are used.
 *
 * To simplify user protocol, special array is used to store all commands (SD_CMDS[]),
 * this array must be the same on both sides of connection.
 *
 * */

/* Packet format */

/* REGULAR MESSAGE [0]cobsencoded([sequence][cmd][size][invchksumm][body0][body1][body2]...[bodyNsize])
 * [0] - COBS_SYMBOL
 * header 4 bytes
 * body >=0 bytes
 * */
typedef struct{
 uint8_t sequence;  // 0-3bits = sequence,4-6bits - not used, 7th bit (0x80) indicate that comfirm  is requested
 uint8_t cmd;       // 0-6bits = cmd, 7th bit indicate GET (0x80) or SET command
 uint8_t size;      // body size
 uint8_t invchksumm;// body inverse check summ
}sd_header_t, *psd_header_t;

/*  SYSTEM MESSAGE FORMAT [0]cobsencoded([sequence][cmd][size][invchksumm])
 *  [0] - COBS_SYMBOL
 *  system messages 4 bytes, never have a body
 *
 * typedef struct{
 *  uint8_t sequence;  // 0-3bits = sequence,4-6bits - system message reason
 *  uint8_t cmd;       // always == SP_SYSTEM_MESSAGE
 *  uint8_t size;      // cmd for which this system mssage generated, without 7th bit
 *  uint8_t invchksumm;// status in system message (any arbitrary value)
 * }sd_header_t, *psd_header_t;
 *
 * system message reason description and invchksumm meaning
 *  SP_OK - delivery confirmation,
 *          invchksumm for set CMD only, contain uint8_t rxcallback answer if cmd supports callback
 *
 *  SP_UNKNOWNCMD - unknown cmd
 *          invchksumm contain body size
 *
 *  SP_WRONGCHECKSUMM - wrong checksumm
 *          invchksumm contain original invchksumm
 *
 *  SP_WRONGSIZE - wrong body size
 *          invchksumm contain original body size
 *
 *  SP_VERSION - report checksum
 *          invchksumm contain checksum of SD_CMDS array
 * */

typedef enum {
  SP_SYSTEM_MESSAGE = 0,
  SP_CONFIGURATION = 1
}SerialPacketType_t;

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

static int16_t build_and_send_package(sd_header_t *hdr,size_t bodysize,uint8_t* body){
  uint16_t pktsize=0,i;
  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;//after cobs_encode data will be starting from [0]
  uint8_t *hdrp=(uint8_t *)hdr;

  if(sd_lock_buffer(500)){//try to lock with timeout

    raw[0]=hdrp[0];//sequence
    raw[1]=hdrp[1];//cmd

    if(bodysize > 0){
      if( body != NULL ){
        uint8_t *raw_body = raw + SD_HEADER_SIZE;
        uint8_t raw_body_checksumm = 0;
        //copy body to temporary buffer, calculate checksumm
        sd_syslock();
          for( i=0; i < bodysize; i++){
            raw_body[i] = body[i];
            raw_body_checksumm += body[i];
          }
          //~ memccpy(,,bodysize,sizeof(uint8_t));
        sd_sysunlock();

        raw[2] = bodysize;//size
        raw[3] = (uint8_t)~raw_body_checksumm;//invchksumm
      }else{
        sd_unlock_buffer();
        return -1; //protocol error, bodysize >0 but body == NULL
      }
    }else{//no body
      raw[2] = hdrp[2];//in size, some usefull data
      raw[3] = hdrp[3];//in invchksumm, some usefull data
    }

    pktsize = bodysize + SD_HEADER_SIZE;//body + header

    pktsize = cobs_encode(raw,pktsize,cobs_buf_p);//move to the left

    if( sd_put_timeout(COBS_SYMBOL,100) == 0 ){//start of packet
      size_t rc = sd_write_timeout(cobs_buf_p,pktsize,500);//send encoded data
      sd_unlock_buffer();
      return (rc == pktsize ? (int16_t)rc : -1 ); //-1 //protocol error
    }else{
      sd_unlock_buffer();
      return -1; //protocol error
    }
  }
    else return -1; //LOCK error

}//build_and_send_package

static int16_t call_callback_build_and_send_package(sd_header_t *hdr,SD_CALLBACK callback){
  uint16_t pktsize=0;
  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;//after cobs_encode data will be starting from [0]
  uint8_t *hdrp=(uint8_t *)hdr;

  if( callback != NULL && sd_lock_buffer(500)){//try to lock with timeout

    uint8_t *raw_body = raw + SD_HEADER_SIZE;
    uint8_t bodysize = MIN(SD_MAX_PACKET,hdr->size);//max packet size
    sd_syslock();
    uint8_t invchksumm = callback(0,raw_body,&bodysize);
    if(invchksumm != 0){//filled data succsessfull
      hdr->size = bodysize;//manually fill header
      hdr->invchksumm = invchksumm;
    }else{
      sd_sysunlock();
      sd_unlock_buffer();
      return -1; //callback didn't return data
    }
    sd_sysunlock();


    raw[0] = hdrp[0];//sequence
    raw[1] = hdrp[1];//cmd
    raw[2] = hdrp[2];//size
    raw[3] = hdrp[3];//invchksumm

    pktsize = bodysize + SD_HEADER_SIZE;//body + header

    pktsize = cobs_encode(raw,pktsize,cobs_buf_p);//move to the left

    if( sd_put_timeout(COBS_SYMBOL,100) == 0 ){//start of packet
      size_t rc = sd_write_timeout(cobs_buf_p,pktsize,500);//send encoded data
      sd_unlock_buffer();
      return (rc == pktsize ? (int16_t)rc : -1 ); //-1 //protocol error
    }else{
      sd_unlock_buffer();
      return -1; //protocol error
    }
  }
    else return -1; //LOCK error

}//build_and_send_package

static size_t cobs_receive_decode(size_t pktsize, uint8_t* destination,uint8_t invchksumm){

  uint8_t *dst = destination;
  uint8_t *end = dst + pktsize;
  uint8_t chksumm = 0;
  int32_t c;


  while(dst < end)
  {
      if(cobs_state_rx.n < 2){ //load new code only if previous was fully parsed (cobs_state_rx.n==1 || cobs_state_rx.n==0)

        cobs_state_rx.code = sd_get_timeout(100);

        if(cobs_state_rx.code < 0 || cobs_state_rx.code == COBS_SYMBOL /*|| ( (cobs_state_rx.read_index + cobs_state_rx.code) > end && cobs_state_rx.code != 1)*/)
            return 0;//error

        cobs_state_rx.n = cobs_state_rx.code;
      }

      while( cobs_state_rx.n > 1 ){
        if(dst < end){

          c = sd_get_timeout(100);

          if(c < 0 || c == COBS_SYMBOL)//COBS_SYMBOL is not allowed there!
            return 0;//error

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
  if( invchksumm !=0 && ((uint8_t)~chksumm) != invchksumm)
    return 0;//error, broken package, checksum mismatch

  return (dst - destination);
}//cobs_receive_decode

static inline size_t cobs_start_receiving_and_decode(size_t pktsize, uint8_t* destination){
  cobs_rx_reset();
  return cobs_receive_decode(pktsize, destination,0);
}


/* skip current packet body because of some error */
static inline void skip(sd_header_t *hdr){
  uint8_t s = hdr->size;
  while( s-- > 0 && sd_get_timeout(100) >=0)
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

static inline void skip_and_aswer(sd_header_t *hdr, SerialPacketSystemMessageReason_t err, uint8_t status){
  skip(hdr);
  system_message_answer(hdr,err,status);
}

uint8_t calculate_version_checksumm(void){
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
static inline void _sd_main_loop_iterate(void){
    int32_t c = -1;
    c = sd_get_timeout(100);

    //~ if(c >=0) sd_put_timeout(c,100);//loopback

    if(c == COBS_SYMBOL){
      /* got packet delimeter,
       * recieve header
       * */
      uint8_t header[SD_HEADER_SIZE]={0};
      sd_header_t *hdr = (sd_header_t *)header;

      //~ sd_wait_for_chars(3);//wait for count of SD_HEADER_SIZE

      if( cobs_start_receiving_and_decode(SD_HEADER_SIZE,header) == SD_HEADER_SIZE ){
        //got header
        if(hdr->cmd == SD_CMD_CREATE_GET(SP_SYSTEM_MESSAGE) ){//special case,"get version"
          system_message_answer(hdr,SP_VERSION,calculate_version_checksumm());
        }else if(hdr->cmd == SP_SYSTEM_MESSAGE){//received protocol information
            sd_broadcast_system_message(hdr->sequence, hdr->size, hdr->invchksumm,500);
            sd_protocol_inform(hdr->sequence, hdr->size, hdr->invchksumm);
        }else{
          last_sequence = SD_SEQ_MASK(hdr->sequence);//used for next outgoing message

          if(hdr->size < SD_MAX_PACKET ){
            //hdr->cmd == SP_SYSTEM_MESSAGE
            if( SD_CMD_INDEX_MASK(hdr->cmd) < SD_CMDS_COUNT ){//known CMD

              if(SD_CMD_ISGET(hdr->cmd)){
                //get CMD
                hdr->cmd = SD_CMD_INDEX_MASK(hdr->cmd);//remove GET bit

                if(SD_CMDS[hdr->cmd].tx_data_size != 0 ){//fixed size format
                  if(SD_CMDS[hdr->cmd].tx_data_size == hdr->size){//check size
                    build_and_send_package(hdr,SD_CMDS[hdr->cmd].tx_data_size,SD_CMDS[hdr->cmd].tx_data);

                    //tx callback
                    if(SD_CMDS[hdr->cmd].tx_callback != NULL){
                      SD_CMDS[hdr->cmd].tx_callback(hdr->size,SD_CMDS[hdr->cmd].tx_data,SD_CMDS[hdr->cmd].tx_arg);
                    }
                  }else{
                    skip_and_aswer(hdr,SP_WRONGSIZE,SD_CMDS[hdr->cmd].tx_data_size);
                  }
                }else{//variable size format, or wrong configuration
                  //call tx callback to fill variable data
                  if(SD_CMDS[hdr->cmd].tx_callback != NULL){
                    call_callback_build_and_send_package(hdr,SD_CMDS[hdr->cmd].tx_callback);
                  }else{
                    skip_and_aswer(hdr,SP_WRONGSIZE,SD_CMDS[hdr->cmd].tx_data_size);//report our prefered size
                  }
                }//end variable size format
              }else{//set CMD
                if( SD_CMDS[hdr->cmd].rx_data_size == hdr->size ){//fixed size format

                  //receive body in temporary buffer
                  if(sd_lock_buffer(500)){
                    if(cobs_receive_decode(hdr->size, cobs_buf_p, hdr->invchksumm) == hdr->size ){

                      //got body
                      uint8_t* rx_data = SD_CMDS[hdr->cmd].rx_data;

                      if(rx_data != NULL){
                        //copy body to destination
                        sd_syslock();
                        uint16_t i;
                        for(i=0; i < hdr->size; i++){
                          rx_data[i]=cobs_buf_p[i];
                        }
                        sd_sysunlock();
                        sd_unlock_buffer();
                      }else
                        rx_data = cobs_buf_p;

                      //callback
                      uint8_t callback_status;
                      if(SD_CMDS[hdr->cmd].rx_callback != NULL){
                        callback_status = SD_CMDS[hdr->cmd].rx_callback(hdr->size,rx_data,SD_CMDS[hdr->cmd].rx_arg);
                      }

                      if(SD_SEQ_ISCONFIRM(hdr->sequence)){ //comfirm requested
                        //send ak
                        system_message_answer(hdr,SP_OK,callback_status);
                      }
                    }else {//body error
                      sd_unlock_buffer();
                      system_message_answer(hdr,SP_WRONGCHECKSUMM,hdr->invchksumm); //don't skip because already partially or completely received
                    }
                  }//sd_lock_buffer

                }else{//variable size format, or wrong configuration
                  //call tx callback to fill variable data
                  if(SD_CMDS[hdr->cmd].rx_callback != NULL){
                    //receive body in temporary buffer
                    if(sd_lock_buffer(500)){
                      if(cobs_receive_decode(hdr->size, cobs_buf_p, hdr->invchksumm) == hdr->size ){
                        //got body
                        uint8_t callback_status = SD_CMDS[hdr->cmd].rx_callback(hdr->size, cobs_buf_p, NULL);
                        sd_unlock_buffer();

                        if(SD_SEQ_ISCONFIRM(hdr->sequence)){ //comfirm requested
                          //send ak
                          system_message_answer(hdr,SP_OK,callback_status);
                        }

                      }else {//body error
                        sd_unlock_buffer();
                        system_message_answer(hdr,SP_WRONGCHECKSUMM,hdr->invchksumm); //don't skip because already partially or completely received
                      }
                    }//sd_lock_buffer
                  }else{
                    skip_and_aswer(hdr,SP_WRONGSIZE,SD_CMDS[hdr->cmd].rx_data_size);//report our prefered size
                  }
                }//end RX variable size format

              }//CMD set
            }else
              skip_and_aswer(hdr,SP_UNKNOWNCMD,hdr->size);

          }else
            skip_and_aswer(hdr,SP_WRONGSIZE,hdr->size);
        }//nonsystem message

      }//if got header

    }//if COBS_SYMBOL
}//_sd_main_loop



void serial_protocol_main_loop_iterate(void){
  _sd_main_loop_iterate();
}

int serial_protocol_get_cmd_async(uint8_t cmd){
  if(cmd < SD_CMDS_COUNT )
  {
    sd_header_t hdr1;
    sd_header_t  *hdr = &hdr1;
    hdr->sequence = SD_SEQ_CREATE(++last_sequence,0); //remove confirm bit, the answer is our confirm
    hdr->cmd = SD_CMD_CREATE_GET(cmd);//GET
    hdr->size = SD_CMDS[cmd].rx_data_size;
    hdr->invchksumm = 0;
    if( (hdr->size != 0 && build_and_send_package(hdr, hdr->size, NULL) == hdr->size) || (hdr->size == 0 && build_and_send_package(hdr, 0, NULL) == (SD_HEADER_SIZE+1)) )
        return SD_SEQ_MASK(hdr->sequence);//OK
    else
        return -1;//ERROR
  }else
    return -1;//ERROR
}

int32_t serial_protocol_get_cmd_sync(uint8_t cmd){
  int sequence = serial_protocol_get_cmd_async(cmd);
  if(sequence < 0)
    return -1;//error
  else
    return sd_wait_system_message(sequence,cmd);//return sd_wait_system_message state, < 0 if error
}



int32_t serial_protocol_set_cmd_sync(uint8_t cmd, uint8_t confirm){
  if(cmd < SD_CMDS_COUNT )
  {
    sd_header_t hdr1;
    sd_header_t  *hdr = &hdr1;
    hdr->sequence = SD_SEQ_CREATE(++last_sequence,confirm);
    hdr->cmd = SD_CMD_CREATE_SET(cmd);//SET

    if(build_and_send_package(hdr,SD_CMDS[hdr->cmd].tx_data_size,SD_CMDS[hdr->cmd].tx_data)<0)
      return -1;//ERROR

    if(confirm){
      return (sd_wait_system_message(hdr->sequence,hdr->cmd)< 0 ? -2 : 0);//-2 - timeout, 0 - successful
    }
    return 0;//OK
  }else
    return -1;//ERROR
}

int32_t serial_protocol_get_cmds_version(void){
    int32_t version_checksumm = serial_protocol_get_cmd_sync(0);//special system message

    if(version_checksumm < 0) return -1;//error, timeout

    return (version_checksumm & 0xFF);
  }

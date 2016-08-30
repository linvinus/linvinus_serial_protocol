
/* Cobs encoding
 *
 * http://www.stuartcheshire.org/papers/COBSforSIGCOMM/
 *
 * */

#define STANDALONE 0

#if STANDALONE
#include <stdlib.h>
#endif

#include "cobs.h"

//0..255 encode to 0..COBS_SYMBOL COBS_SYMBOL..255 (exclude COBS_SYMBOL)

size_t cobs_encode(const uint8_t* source, size_t size, uint8_t* destination)
{
    size_t read_index  = 0;
    size_t write_index = 1;
    size_t code_index  = 0;
    uint8_t code       = 1;

    while(read_index < size)
    {
        if(source[read_index] == COBS_SYMBOL)
        {
            destination[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        }
        else
        {
            destination[write_index++] = source[read_index++];
            code++;

            if(code == 0xFF)
            {
                destination[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }

    destination[code_index] = code;

    return write_index;
}

/*
 * destination must be a size of cobs_get_encoded_buffer_size(size) or more
 * */
size_t cobs_decode(const uint8_t* source, size_t size, uint8_t* destination)
    {
        size_t read_index  = 0;
        size_t write_index = 0;
        uint8_t code;
        uint8_t i;

        while(read_index < size)
        {
            code = source[read_index];

            if(read_index + code > size && code != 1)
            {
                return 0;
            }

            read_index++;

            for(i = 1; i < code; i++)
            {
                destination[write_index++] = source[read_index++];
            }

            if(code != 0xFF && read_index != size)
            {
                destination[write_index++] = COBS_SYMBOL;
            }
        }

        return write_index;
}

size_t cobs_get_encoded_buffer_size(size_t sourceSize)
{
    return sourceSize + sourceSize / 254 + 1;
}
#if STANDALONE

typedef struct{
  int32_t n;    //current processed byte
  int32_t code; //last code
} cobs_state_t;

static cobs_state_t cobs_state_rx; //store state between calls of cobs_receive_decode

static size_t cobs_receive_decode(size_t pktsize, uint8_t* destination,uint8_t invchksumm, uint8_t* source){

  uint8_t *dst = destination;
  uint8_t *end = dst + pktsize;
  uint8_t chksumm = 0;
  int32_t c;


  while(dst < end)
  {
      if(cobs_state_rx.n < 1){ //load new code only if previous was fully parsed (cobs_state_rx.n==1 || cobs_state_rx.n==0)

        cobs_state_rx.code = *(source++);

        if(cobs_state_rx.code < 0 || cobs_state_rx.code == COBS_SYMBOL /*|| ( (cobs_state_rx.read_index + cobs_state_rx.code) > end && cobs_state_rx.code != 1)*/)
            return -1;//error

        cobs_state_rx.n = cobs_state_rx.code;
      }

      while( cobs_state_rx.n > 1 ){
        if(dst < end){

          c = *(source++);

          if(c < 0 || c == COBS_SYMBOL)//COBS_SYMBOL is not allowed there!
            return -1;//error

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
    return -1;//error, broken package, checksum mismatch

  return (dst - destination);
}//cobs_receive_decode


int main(void) {
  int i;
  //~ uint8_t data[]={0x45,0x00,0x00,0x2c,0x4c,0x79,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x06,0x4f,0x37};
  uint8_t data[]={0xc, 0x1, 0x10, 0xf8, 0x0, 0x1, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0};
  int newsize=cobs_get_encoded_buffer_size(sizeof(data));
  uint8_t dst[newsize];
  uint8_t dst2[newsize];
  cobs_encode(data,sizeof(data),dst);
  //~ int newsize2 = cobs_decode(dst,sizeof(dst),dst2);
  int newsize2 = cobs_receive_decode(4,dst2,0,dst);
  newsize2 = cobs_receive_decode(16,dst2+4,0,dst+newsize2+1);
  printf("size of data=%d, size of dst=%d \r\n",sizeof(data),newsize);
  for(i=0; i<sizeof(data); i++){
    printf("0x%02x ",data[i]);
  }
  printf("\r\n");
  for(i=0; i<sizeof(dst); i++){
    printf("0x%02x ",dst[i]);
  }
  printf("\r\n");
  for(i=0; i<newsize2; i++){
    printf("0x%02x ",dst2[i]);
  }
  printf("\r\n");
}
#endif

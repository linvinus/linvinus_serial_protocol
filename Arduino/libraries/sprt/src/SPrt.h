/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#ifndef SPrt_h
#define SPrt_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "serial_protocol.h"

#if defined(UBRR1H)
#define sptr_serial  (&Serial1)
#endif

#if defined(UBRR2H)
#define sptr_serial (&Serial2)
#endif

#if defined(UBRR3H)
#define sptr_serial (&Serial3)
#endif

#ifndef sptr_serial
#define sptr_serial (&Serial)
#endif





class SPrt
{
  public:
    SPrt()
    {
    }

    ~SPrt()
    {
    }

    void begin(unsigned long baud, size_t port = 0)
        {
            //~ switch(port)
            //~ {
            //~ #if defined(UBRR1H)
                //~ case 1:
                    //~ Serial1.begin(baud);
                    //~ _serial = &Serial1;
                    //~ break;
            //~ #endif
            //~ #if defined(UBRR2H)
                //~ case 2:
                    //~ Serial2.begin(baud);
                    //~ _serial = &Serial2;
                    //~ break;
            //~ #endif
            //~ #if defined(UBRR3H)
                //~ case 3:
                    //~ Serial3.begin(baud);
                    //~ _serial = &Serial3;
                    //~ break;
            //~ #endif
                //~ default:
                    //~ Serial.begin(baud);
                    //~ _serial = &Serial;
            //~ }
            sptr_serial->begin(baud);
    }

    //~ void begin(Stream* serial)
    //~ {
        //~ _serial = serial;
    //~ }

    void update()
    {
      if (sptr_serial == 0) return;
      if(sptr_serial->available() > 0){
        _sprt_main_loop_iterate();
      }
    }

    void set_print_cmd(uint8_t cmd){
      _print_cmd = cmd;
    }

    int32_t printf(uint8_t cmd,const char *fmt,...){
      if(cmd == 0) return -1;
      int32_t ret;
      va_list ap;
      va_start(ap, fmt);
      ret = sprt_vsprintf(cmd,fmt,ap);
      va_end(ap);
      return ret;
    }

    int32_t printf(const char *fmt,...){
      if(_print_cmd == 0) return -1;
      int32_t ret;
      va_list ap;
      va_start(ap, fmt);
      ret = sprt_vsprintf(_print_cmd,fmt,ap);
      va_end(ap);
      return ret;
    }

    size_t write(const char *str) {
      if (str == NULL || _print_cmd == 0) return -1;
      return send(_print_cmd,(uint8_t *)str, strlen(str),false);
    }

    size_t write(const char *buffer, size_t size) {
      if (buffer == NULL || _print_cmd == 0 || size == 0) return -1;
      return send(_print_cmd,(uint8_t *)buffer, size,false);
    }

    int32_t send(uint8_t cmd,uint8_t *buff,size_t count,uint8_t confirm){
      return _sprt_send_with_data(cmd,buff,count,confirm);
    }

    int32_t send(uint8_t cmd, bool confirm){
      return sprt_send(cmd, confirm);
    }

    int32_t receive(uint8_t cmd, bool confirm){
      return sprt_receive(cmd,confirm);
    }


    int32_t exchange(uint8_t cmd, bool confirm){
      return sprt_exchange(cmd, confirm);
    }

  private:
    //~ Stream* _serial;
    uint8_t _print_cmd=0;
};

#endif //SPrt_h


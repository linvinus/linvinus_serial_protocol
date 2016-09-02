#ifndef _SERIAL_PROTOCOL_LLD_H_
#define _SERIAL_PROTOCOL_LLD_H_


#ifdef __cplusplus
extern "C" {
#endif

  extern uint8_t cobs_buf1[SD_BUFFER_LENGTH];

#ifdef __cplusplus
}
#endif

//typedef uint8_t size_t;
#include "stddef.h" //size_t
#include <stdarg.h> /*va_list for sprt_lld_sprintf*/
//#include <time.h>   /*for struct timespec*/

#define sprt_lld_lock_buffer(a) (1)
#define sprt_lld_unlock_buffer(a) (1)
#define sprt_lld_syslock()
#define sprt_lld_sysunlock()
#define sprt_lld_wait_for_chars(s)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

     int sprt_thread_init();

     int sprt_lld_put_timeout(char b,int time_ms);
     int sprt_lld_get_timeout(int time_ms);
     int sprt_lld_write_timeout(uint8_t *buff,int size,int time_ms);
//~ uint16_t sprt_lld_lock_buffer(uint32_t time_ms);
//~ uint16_t sprt_lld_unlock_buffer(void);
     int sprt_lld_sprintf(uint8_t *str, size_t size, const char *fmt,va_list ap);
    //void sprt_lld_timespec_diff(struct timespec *start, struct timespec *stop, struct timespec *result);
    void sprt_lld_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state,uint32_t timeout_ms);
    void sprt_lld_flush(void);

   /*from serial_protocol.c*/
     void _sprt_main_loop_iterate(void);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /*_SERIAL_PROTOCOL_LLD_H_*/

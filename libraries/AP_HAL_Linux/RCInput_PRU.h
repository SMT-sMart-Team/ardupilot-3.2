
#ifndef __AP_HAL_LINUX_RCINPUT_PRU_H__
#define __AP_HAL_LINUX_RCINPUT_PRU_H__

/*
  This class implements RCInput on the BeagleBoneBlack with a PRU
  doing the edge detection of the PPM sum input
 */

#include <AP_HAL_Linux.h>

// #define RCIN_PRUSS_SHAREDRAM_BASE   0x4a312000
#define RCIN_PRUSS_SHAREDRAM_BASE   0x4a311000
// we use 300 ring buffer entries to guarantee that a full 25 byte
// frame of 12 bits per byte
//
// AB ZhaoYJ for trying to add PPMSUM decoding in PRU @2016-05-21
// #define PPMSUM_DECODE_IN_PRU
#define MAX_RCIN_NUM 16
#define OK 0xbeef
#define KO 0x4110 /// !beef

class Linux::LinuxRCInput_PRU : public Linux::LinuxRCInput 
{
public:
    void init(void*);
    void _timer_tick(void);

 private:
    static const unsigned int NUM_RING_ENTRIES=256;
    // static const unsigned int NUM_RING_ENTRIES=300;
    // shared ring buffer with the PRU which records pin transitions
    struct rb{
        volatile uint16_t ring_head; // owned by ARM CPU
        volatile uint16_t ring_tail; // owned by the PRU
        volatile struct {
               volatile uint16_t pin_value;
               volatile uint16_t delta_t;
        } buffer[NUM_RING_ENTRIES];
#ifdef PPMSUM_DECODE_IN_PRU
        volatile struct {
               volatile uint16_t rcin_value[MAX_RCIN_NUM];
               volatile uint16_t new_rc_input;
               volatile uint16_t _num_channels;
        } ppm_decode_out;
#endif
    };
    volatile struct rb *ring_buffer;


#ifdef SMT_NEW_RCIN
    struct rb rb_local;
#endif

    // time spent in the low state
    uint16_t _s0_time;
};

#endif // __AP_HAL_LINUX_RCINPUT_PRU_H__


#define CONFIG_HAL_BOARD HAL_BOARD_LINUX

#include "RCInput_PRU_test.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>


#ifdef SMT_NEW_RCIN


void LinuxRCInput_PRU::init(void*)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        hal.scheduler->panic("Unable to open /dev/mem");
    }
#ifdef PRU_SHM_NEW_SIZE
    ring_buffer = (volatile struct rb*) mmap(0, sizeof(rb) + 0x100, PROT_READ|PROT_WRITE, 
                                                      MAP_SHARED, mem_fd, RCIN_PRUSS_SHAREDRAM_BASE);
#else
    ring_buffer = (volatile struct rb*) mmap(0, 0x1000,PROT_READ|PROT_WRITE, 
                                                    MAP_SHARED, mem_fd, RCIN_PRUSS_SHAREDRAM_BASE);
#endif

    if(MAP_FAILED == ring_buffer)
    {
        hal.scheduler->panic("Failed to mmap PRU0 SHM\n");
    }
    close(mem_fd);
    ring_buffer->ring_head = 0;
    rb_local.ring_head = 0;
    _s0_time = 0;
}

#ifdef PPMSUM_DECODE_IN_PRU

// get rcin values of PPM
void LinuxRCInput_PRU::_timer_tick()
{
    uint16_t *dst = NULL;
    uint16_t *src = NULL;
    uint16_t len = 0;

    // do PPM first for nothing to do with decoding
    if(OK == ring_buffer->ppm_decode_out.new_rc_input)
    {
        ring_buffer->ppm_decode_out.new_rc_input = KO;
        memcpy((void*)_pwm_values, (void*)ring_buffer->ppm_decode_out.rcin_value, sizeof(uint16_t)*MAX_RCIN_NUM); 
        // for(unsigned ii = 0; ii < MAX_RCIN_NUM; ii++)
        // {
        //     if(_pwm_values[ii] == 1929)
        //     {
        //         printf("OMG: ch%d is %d\n", ii, _pwm_values[ii]);
        //     }
        // }
        _num_channels = ring_buffer->ppm_decode_out._num_channels;
        new_rc_input = true;
    }

    // first, copy all new rcinput 
    rb_local.ring_tail = ring_buffer->ring_tail;
    if (rb_local.ring_tail >= NUM_RING_ENTRIES) {
        // invalid ring_tail from PRU - ignore RC input
        return;
    }
    if(rb_local.ring_tail != rb_local.ring_head)
    {
        // copy once
        if(rb_local.ring_tail > rb_local.ring_head)
        {
            dst = (uint16_t *)&(rb_local.buffer[rb_local.ring_head]);
            src = (uint16_t *)ring_buffer->buffer + rb_local.ring_head*2;
            len = (rb_local.ring_tail - rb_local.ring_head)*2;
            memcpy((void*)dst, (void*)src, len*2);
        }
        else // copy twice
        {
            
            // 1
            dst = (uint16_t *)&(rb_local.buffer[rb_local.ring_head]);
            src = (uint16_t *)ring_buffer->buffer + rb_local.ring_head*2;
            len = (NUM_RING_ENTRIES - rb_local.ring_head)*2;
            memcpy((void*)dst, (void*)src, len*2);

            // 2
            dst = (uint16_t *)rb_local.buffer;
            src = (uint16_t *)ring_buffer->buffer;
            len = rb_local.ring_tail*2;
            memcpy((void*)dst, (void*)src, len*2);
        }

        // decoding
        while (rb_local.ring_head != rb_local.ring_tail) {
            if (rb_local.ring_tail >= NUM_RING_ENTRIES) {
                // invalid ring_tail from PRU - ignore RC input
                return;
            }
            if (rb_local.buffer[rb_local.ring_head].pin_value == 1) {
                // remember the time we spent in the low state
                _s0_time = rb_local.buffer[rb_local.ring_head].delta_t;
            } else {
                // the pulse value is the sum of the time spent in the low
                // and high states

                // treat as SBUS
                _process_sbus_pulse(_s0_time, rb_local.buffer[rb_local.ring_head].delta_t);

                // treat as DSM
                _process_dsm_pulse(_s0_time, rb_local.buffer[rb_local.ring_head].delta_t);
            }
            // move to the next ring buffer entry
            ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;        
            rb_local.ring_head = (rb_local.ring_head + 1) % NUM_RING_ENTRIES;        
        }
    }


}
#else
/*
  called at 1kHz to check for new pulse capture data from the PRU
 */
void LinuxRCInput_PRU::_timer_tick()
{
    uint16_t *dst = NULL;
    uint16_t *src = NULL;
    uint16_t len = 0;
    // first, copy all new rcinput 
    rb_local.ring_tail = ring_buffer->ring_tail;
    if (rb_local.ring_tail >= NUM_RING_ENTRIES) {
        // invalid ring_tail from PRU - ignore RC input
        return;
    }
    if(rb_local.ring_tail != rb_local.ring_head)
    {
        // copy once
        if(rb_local.ring_tail > rb_local.ring_head)
        {
            dst = (uint16_t *)&(rb_local.buffer[rb_local.ring_head]);
            src = (uint16_t *)ring_buffer->buffer + rb_local.ring_head*2;
            len = (rb_local.ring_tail - rb_local.ring_head)*2;
            memcpy((void*)dst, (void*)src, len*2);
        }
        else // copy twice
        {
            
            // 1
            dst = (uint16_t *)&(rb_local.buffer[rb_local.ring_head]);
            src = (uint16_t *)ring_buffer->buffer + rb_local.ring_head*2;
            len = (NUM_RING_ENTRIES - rb_local.ring_head)*2;
            memcpy((void*)dst, (void*)src, len*2);

            // 2
            dst = (uint16_t *)rb_local.buffer;
            src = (uint16_t *)ring_buffer->buffer;
            len = rb_local.ring_tail*2;
            memcpy((void*)dst, (void*)src, len*2);
        }

        // decoding
        while (rb_local.ring_head != rb_local.ring_tail) {
            if (rb_local.ring_tail >= NUM_RING_ENTRIES) {
                // invalid ring_tail from PRU - ignore RC input
                return;
            }
            if (rb_local.buffer[rb_local.ring_head].pin_value == 1) {
                // remember the time we spent in the low state
                _s0_time = rb_local.buffer[rb_local.ring_head].delta_t;
            } else {
                // the pulse value is the sum of the time spent in the low
                // and high states
                _process_rc_pulse(_s0_time, rb_local.buffer[rb_local.ring_head].delta_t);
            }
            // move to the next ring buffer entry
            ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;        
            rb_local.ring_head = (rb_local.ring_head + 1) % NUM_RING_ENTRIES;        
        }
    }
}
#endif

#else
// state of ppm decoder
struct {
    int8_t _channel_counter;
    uint16_t _pulse_capt[LINUX_RC_INPUT_NUM_CHANNELS];
} ppm_state;

void LinuxRCInput_PRU::init(void*)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        printf("Unable to open /dev/mem");
    }
    ring_buffer = (volatile struct rb*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, 
                                                      MAP_SHARED, mem_fd, RCIN_PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
    _s0_time = 0;

    ppm_state._channel_counter = -1;

    // enable the spektrum RC input power
    // hal.gpio->pinMode(BBB_P8_17, HAL_GPIO_OUTPUT);
    // hal.gpio->write(BBB_P8_17, 1);
}

/*
  process a PPM-sum pulse of the given width
 */
void LinuxRCInput_PRU::_process_ppmsum_pulse(uint16_t width_usec)
{
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (ppm_state._channel_counter >= 5) {
            for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
                // _pwm_values[i] = ppm_state._pulse_capt[i];
            }
            // _num_channels = ppm_state._channel_counter;
            // new_rc_input = true;
        }
        ppm_state._channel_counter = 0;
        return;
    }
    if (ppm_state._channel_counter == -1) {
        // we are not synchronised
        return;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

        // move to next channel
        ppm_state._channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (ppm_state._channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
        for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
            // _pwm_values[i] = ppm_state._pulse_capt[i];
        }
        // _num_channels = ppm_state._channel_counter;
        // new_rc_input = true;
        ppm_state._channel_counter = -1;
    }
}

#define DUMP_RCIN
#define DUMP_LEN 100000 // 120000
#ifdef DUMP_RCIN
uint16_t log1[DUMP_LEN];
uint16_t log2[DUMP_LEN];
uint16_t log3[DUMP_LEN];
#endif
void LinuxRCInput_PRU::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
#ifdef DUMP_RCIN
	static int cnt = 0;

	if(cnt < DUMP_LEN)
	{
		log1[cnt] = width_s0;
		log2[cnt] = width_s1;
		log3[cnt] = ppm_state._channel_counter;
		cnt ++;
        if(!(cnt%(DUMP_LEN/20)))
        {
		    printf("logging... cnt = %d\n", cnt);
        }
	}
	else
	{
		FILE *rclog = NULL;
		if (rclog == NULL) {
			rclog = fopen("/tmp/rcin.dump", "w");
		}
		{
			if (rclog) {
		        for(int i = 0;i< DUMP_LEN;i++)
                {
			    	fprintf(rclog, "%u,%u,%u\n", (unsigned)log3[i],(unsigned)log1[i], (unsigned)log2[i]);
                }
			}
		}
		printf("log finished \n");
		exit(1);

	}
#endif
    _process_ppmsum_pulse(width_s0 + width_s1);
}

void LinuxRCInput_PRU::_timer_tick()
{
    while (ring_buffer->ring_head != ring_buffer->ring_tail) {
        if (ring_buffer->ring_tail >= NUM_RING_ENTRIES) {
            // invalid ring_tail from PRU - ignore RC input
            return;
        }
        if (ring_buffer->buffer[ring_buffer->ring_head].pin_value == 1) {
            // remember the time we spent in the low state
            _s0_time = ring_buffer->buffer[ring_buffer->ring_head].delta_t;
        } else {
            // the pulse value is the sum of the time spent in the low
            // and high states
            _process_rc_pulse(_s0_time, ring_buffer->buffer[ring_buffer->ring_head].delta_t);
        }
        // move to the next ring buffer entry
        ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;        
    }
}

LinuxRCInput_PRU pruintest;



int main(void)
{
    unsigned int ii = 0;
    printf("enter pruin_test...\n");
    pruintest.init(NULL);

    // first write magic head to pwmpru, then wait for resp
    // then change mask for enable
    while(1)
    {

        pruintest._timer_tick();
        // usleep(200);
    }
}
#endif


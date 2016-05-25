#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
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

#include "RCInput.h"
#include "sbus.h"
#include "dsm.h"

// #define DEBUG_PPM

extern const AP_HAL::HAL& hal;

using namespace Linux;

LinuxRCInput::LinuxRCInput() :
    new_rc_input(false)
{
    ppm_state._channel_counter = -1;
}

void LinuxRCInput::init(void* machtnichts)
{
}

bool LinuxRCInput::new_input() 
{
    return new_rc_input;
}

uint8_t LinuxRCInput::num_channels() 
{
    return _num_channels;
}

uint16_t LinuxRCInput::read(uint8_t ch) 
{
#ifdef DEBUG_PPM
    static uint16_t last[16];
    static uint16_t cnt = 0;
    uint8_t ii = 0;
#endif
    new_rc_input = false;
    if (_override[ch]) {
        return _override[ch];
    }
    if (ch >= _num_channels) {
        return 0;
    }
#ifdef DEBUG_PPM
    // if(2 == ch)
    {
        // if(last[ch] != _pwm_values[ch])
        if(!(cnt%50))
        {
            for(ii = 0; ii < LINUX_RC_INPUT_NUM_CHANNELS; ii++)
            {
                printf("RC%din: %d->%d\n", ii, last[ii], _pwm_values[ii]);
                last[ii] = _pwm_values[ii];
                // _pwm_values[ii] = 0x789;
            }
            cnt = 0;
        }
        cnt++;
    }
#endif
    return _pwm_values[ch];
}

uint8_t LinuxRCInput::read(uint16_t* periods, uint8_t len) 
{
    uint8_t i;
    for (i=0; i<len; i++) {
        periods[i] = read(i);
    }
#ifdef KILL_ERROR_PULSE
    _kill_error_pulse(periods);
#endif
    return (i+1);
}

bool LinuxRCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
    bool res = false;
    if(len > LINUX_RC_INPUT_NUM_CHANNELS){
        len = LINUX_RC_INPUT_NUM_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool LinuxRCInput::set_override(uint8_t channel, int16_t override) 
{
    if (override < 0) return false; /* -1: no change. */
    if (channel < LINUX_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            new_rc_input = true;
            return true;
        }
    }
    return false;
}

void LinuxRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
       _override[i] = 0;
    }
}

#ifdef KILL_ERROR_PULSE
#define ERR_NUM_TH 5
#define NUM_CH 8
#define GLITCH_RCIN 1178 // 1.15 = 1178/1024
void LinuxRCInput::_kill_error_pulse(uint16_t *rc_in)
{
    static uint16_t prev_rc_in[NUM_CH];
    static uint16_t pulse_num[NUM_CH];
    static uint16_t first = 1;
    if(!first)
    {
        for(unsigned ii = 0; ii < NUM_CH; ii++)
        {
            if(rc_in[ii] > ((prev_rc_in[ii]*GLITCH_RCIN) >> 10)) // 1.15
            {
                pulse_num[ii]++;
                if(pulse_num[ii] > ERR_NUM_TH) // normal, let it go
                {
                    pulse_num[ii] = 0;;
                    prev_rc_in[ii] = rc_in[ii];
                }
                else // error
                {
                    rc_in[ii] = prev_rc_in[ii];
                }
            }
        }
    }
    else
    {
        memcpy((void*)prev_rc_in, (void*)rc_in, sizeof(uint16_t)*NUM_CH); 
        first = 0;
    }

}
#endif

/*
  process a PPM-sum pulse of the given width
 */
void LinuxRCInput::_process_ppmsum_pulse(uint16_t width_usec)
{
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (ppm_state._channel_counter >= 5) {
            for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
                _pwm_values[i] = ppm_state._pulse_capt[i];
            }
            _num_channels = ppm_state._channel_counter;
            new_rc_input = true;
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
            _pwm_values[i] = ppm_state._pulse_capt[i];
        }
        _num_channels = ppm_state._channel_counter;
        new_rc_input = true;
        ppm_state._channel_counter = -1;
    }
}

/*
  process a SBUS input pulse of the given width
 */
void LinuxRCInput::_process_sbus_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    uint16_t bits_s0 = (width_s0+1) / 10;
    uint16_t bits_s1 = (width_s1+1) / 10;
    uint16_t nlow;

    uint8_t byte_ofs = sbus_state.bit_ofs/12;
    uint8_t bit_ofs = sbus_state.bit_ofs%12;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }
	
    if (bits_s0+bit_ofs > 10) {
        // invalid data as last two bits must be stop bits
        goto reset;
    }

    // pull in the high bits
    sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
    sbus_state.bit_ofs += bits_s0;
    bit_ofs += bits_s0;

    // pull in the low bits
    nlow = bits_s1;
    if (nlow + bit_ofs > 12) {
        nlow = 12 - bit_ofs;
    }
    bits_s1 -= nlow;
    sbus_state.bit_ofs += nlow;

    if (sbus_state.bit_ofs == 25*12 && bits_s1 > 12) {
        // we have a full frame
        uint8_t bytes[25];
        uint8_t i;
        for (i=0; i<25; i++) {
            // get inverted data
            uint16_t v = ~sbus_state.bytes[i];
            // check start bit
            if ((v & 1) != 0) {
                goto reset;
            }
            // check stop bits
            if ((v & 0xC00) != 0xC00) {
                goto reset;
            }
            // check parity
            uint8_t parity = 0, j;
            for (j=1; j<=8; j++) {
                parity ^= (v & (1U<<j))?1:0;
            }
            if (parity != (v&0x200)>>9) {
                goto reset;
            }
            bytes[i] = ((v>>1) & 0xFF);
        }
        uint16_t values[LINUX_RC_INPUT_NUM_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe=false, sbus_frame_drop=false;
        if (sbus_decode(bytes, values, &num_values, 
                        &sbus_failsafe, &sbus_frame_drop, 
                        LINUX_RC_INPUT_NUM_CHANNELS) && 
            num_values >= 5) {
            for (i=0; i<num_values; i++) {
                _pwm_values[i] = values[i];
            }
            _num_channels = num_values;
            new_rc_input = true;
        }
        goto reset;
    } else if (bits_s1 > 12) {
        // break
        goto reset;
    }
    return;
reset:
    memset(&sbus_state, 0, sizeof(sbus_state));        
}

void LinuxRCInput::_process_dsm_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 115200 bps
    uint16_t bits_s0 = ((width_s0+4)*(uint32_t)115200) / 1000000;
    uint16_t bits_s1 = ((width_s1+4)*(uint32_t)115200) / 1000000;
    uint8_t bit_ofs, byte_ofs;
    uint16_t nbits;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;
    
    if(byte_ofs > 15) {
        // invalid data
        goto reset;
    }

    // pull in the high bits
    nbits = bits_s0;
    if (nbits+bit_ofs > 10) {
        nbits = 10 - bit_ofs;
    }
    dsm_state.bytes[byte_ofs] |= ((1U<<nbits)-1) << bit_ofs;
    dsm_state.bit_ofs += nbits;
    bit_ofs += nbits;

    if (bits_s0 - nbits > 10) {
        if (dsm_state.bit_ofs == 16*10) {
            // we have a full frame
            uint8_t bytes[16];
            uint8_t i;
            for (i=0; i<16; i++) {
                // get raw data
                uint16_t v = dsm_state.bytes[i];
                
                // check start bit
                if ((v & 1) != 0) {
                    goto reset;
                }
                // check stop bits
                if ((v & 0x200) != 0x200) {
                    goto reset;
                }
                bytes[i] = ((v>>1) & 0xFF);
            }
            uint16_t values[8];
            uint16_t num_values=0;
            if (dsm_decode(hal.scheduler->micros64(), bytes, values, &num_values, 8) && 
                num_values >= 5) {
                for (i=0; i<num_values; i++) {
                    _pwm_values[i] = values[i];
                }
                _num_channels = num_values;                
                new_rc_input = true;
            }
        }
        memset(&dsm_state, 0, sizeof(dsm_state));
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;

    if (bits_s1+bit_ofs > 10) {
        // invalid data
        goto reset;
    }

    // pull in the low bits
    dsm_state.bit_ofs += bits_s1;
    return;
reset:
    memset(&dsm_state, 0, sizeof(dsm_state));        
}

/*
  process a RC input pulse of the given width
 */
void LinuxRCInput::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
#ifdef DUMP_RCIN
	static int cnt = 0;
	static uint16_t log1[120000];
	static uint16_t log2[120000];
	static uint16_t log3[120000];

	if(cnt < 120000)
	{
		log1[cnt] = width_s0;
		log2[cnt] = width_s1;
		log3[cnt] = ppm_state._channel_counter;
		cnt ++;
	}
	else
	{
		FILE *rclog;
		if (rclog == NULL) {
			rclog = fopen("/tmp/rcin1.log", "w");
		}
		for(int i = 0;i< 120000;i++)
		{
			if (rclog) {
				fprintf(rclog, "%u,%u,%u\n", (unsigned)log3[i],(unsigned)log1[i], (unsigned)log2[i]);
			}
		}
		printf("log finished \n");
		exit(1);

	}
#endif
    // treat as PPM-sum
    _process_ppmsum_pulse(width_s0 + width_s1);

    // treat as SBUS
    _process_sbus_pulse(width_s0, width_s1);

    // treat as DSM
    _process_dsm_pulse(width_s0, width_s1);
}

#endif // CONFIG_HAL_BOARD

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// @file	LowPassFilter.cpp
/// @brief	A class to implement a second order low pass filter 
/// Author: Leonard Hall <LeonardTHall@gmail.com>

#ifdef NEW_LPF
#include "LowPassFilter2p.h"


////////////////////////////////////////////////////////////////////////////////////////////
// DigitalBiquadFilter
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalBiquadFilter<T>::DigitalBiquadFilter() {
  _delay_element_1 = T();
  _delay_element_2 = T();
}

template <class T>
T DigitalBiquadFilter<T>::apply(const T &sample, const struct biquad_params &params) {
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
        return sample;
    }

    T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

template <class T>
void DigitalBiquadFilter<T>::reset() { 
    _delay_element_1 = _delay_element_2 = T();
}

template <class T>
void DigitalBiquadFilter<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;

    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(PI/fr);
    float c = 1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;

    ret.b0 = ohm*ohm/c;
    ret.b1 = 2.0f*ret.b0;
    ret.b2 = ret.b0;
    ret.a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret.a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
}


////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter2p
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
LowPassFilter2p<T>::LowPassFilter2p() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
LowPassFilter2p<T>::LowPassFilter2p(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void LowPassFilter2p<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    DigitalBiquadFilter<T>::compute_params(sample_freq, cutoff_freq, _params);
}

// return the cutoff frequency
template <class T>
float LowPassFilter2p<T>::get_cutoff_freq(void) const {
    return _params.cutoff_freq;
}

template <class T>
float LowPassFilter2p<T>::get_sample_freq(void) const {
    return _params.sample_freq;
}

template <class T>
T LowPassFilter2p<T>::apply(const T &sample) {
    return _filter.apply(sample, _params);
}

template <class T>
void LowPassFilter2p<T>::reset(void) {
    return _filter.reset();
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter2p<int>;
template class LowPassFilter2p<long>;
template class LowPassFilter2p<float>;
template class LowPassFilter2p<Vector2f>;
template class LowPassFilter2p<Vector3f>;

#else

#include <inttypes.h>
#include <AP_Math.h>
#include "LowPassFilter2p.h"

void LowPassFilter2p::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
    _cutoff_freq = cutoff_freq;
    float fr = sample_freq/_cutoff_freq;
    float ohm = tanf(PI/fr);
    float c = 1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
    _b0 = ohm*ohm/c;
    _b1 = 2.0f*_b0;
    _b2 = _b0;
    _a1 = 2.0f*(ohm*ohm-1.0f)/c;
    _a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
}

float LowPassFilter2p::apply(float sample)
{
    // do the filtering
    float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
    
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}
#endif
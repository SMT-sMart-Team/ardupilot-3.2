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

//
/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

#ifndef __LOW_PASS_FILTER_H__
#define __LOW_PASS_FILTER_H__


#ifdef NEW_LPF
#include <AP_Math.h>
#include "FilterClass.h"

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:
    struct lpf_params {
        float cutoff_freq;
        float sample_freq;
        float alpha;
    };  

    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, float cutoff_freq, float dt);
    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;
    void reset(T value);

private:
    T _output;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(float cutoff_freq);

    // change parameters
    void set_cutoff_frequency(float cutoff_freq);
    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    T apply(T sample, float dt);
    const T &get() const;
    void reset(T value);
    
protected:
    float _cutoff_freq;

private:
    DigitalLPF<T> _filter;
};

// Uncomment this, if you decide to remove the instantiations in the implementation file
/*
template <class T>
LowPassFilter<T>::LowPassFilter() : _cutoff_freq(0.0f) { 
  
}
// constructor
template <class T>
LowPassFilter<T>::LowPassFilter(float cutoff_freq) : _cutoff_freq(cutoff_freq) { 
  
}
*/

// typedefs for compatibility
typedef LowPassFilter<int>      LowPassFilterInt;
typedef LowPassFilter<long>     LowPassFilterLong;
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Vector2f> LowPassFilterVector2f;
typedef LowPassFilter<Vector3f> LowPassFilterVector3f;


#else
#include <AP_Math.h>
#include "FilterClass.h"
// 1st parameter <T> is the type of data being filtered.
template <class T>
class LowPassFilter : public Filter<T>
{
public:
    // constructor
    LowPassFilter();

    void set_cutoff_frequency(float time_step, float cutoff_freq);
    void set_time_constant(float time_step, float time_constant);

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T        apply(T sample);

    // reset - clear the filter - next sample added will become the new base value
    virtual void        reset() {
        _base_value_set = false;
    };

    // reset - clear the filter and provide the new base value
    void        reset( T new_base_value ) {
        _base_value = new_base_value; _base_value_set = true;
    };

private:
    float           _alpha;             // gain value  (like 0.02) applied to each new value
    bool            _base_value_set;    // true if the base value has been set
    float           _base_value;        // the number of samples in the filter, maxes out at size of the filter
};

// Typedef for convenience (1st argument is the data type, 2nd is a larger datatype to handle overflows, 3rd is buffer size)
typedef LowPassFilter<int8_t> LowPassFilterInt8;
typedef LowPassFilter<uint8_t> LowPassFilterUInt8;

typedef LowPassFilter<int16_t> LowPassFilterInt16;
typedef LowPassFilter<uint16_t> LowPassFilterUInt16;

typedef LowPassFilter<int32_t> LowPassFilterInt32;
typedef LowPassFilter<uint32_t> LowPassFilterUInt32;

typedef LowPassFilter<float> LowPassFilterFloat;

// Constructor    //////////////////////////////////////////////////////////////

template <class T>
LowPassFilter<T>::LowPassFilter() :
    Filter<T>(),
    _alpha(1),
    _base_value_set(false)
{};

//    F_Cut = 1; % Hz
//RC = 1/(2*pi*F_Cut);
//Alpha = Ts/(Ts + RC);

// Public Methods //////////////////////////////////////////////////////////////

template <class T>
void LowPassFilter<T>::set_cutoff_frequency(float time_step, float cutoff_freq)
{
    // avoid divide by zero and allow removing filtering
    if (cutoff_freq <= 0.0f) {
        _alpha = 1.0f;
        return;
    }

    // calculate alpha
    float rc = 1/(2*PI*cutoff_freq);
    _alpha = time_step / (time_step + rc);
}

template <class T>
void LowPassFilter<T>::set_time_constant(float time_step, float time_constant)
{
    // avoid divide by zero
    if (time_constant + time_step <= 0.0f) {
        _alpha = 1.0f;
        return;
    }

    // calculate alpha
    _alpha = time_step / (time_constant + time_step);
}

template <class T>
T LowPassFilter<T>::apply(T sample)
{
    // initailise _base_value if required
    if( !_base_value_set ) {
        _base_value = sample;
        _base_value_set = true;
    }

    // do the filtering
    //_base_value = _alpha * (float)sample + (1.0 - _alpha) * _base_value;
    _base_value = _base_value + _alpha * ((float)sample - _base_value);

    // return the value.  Should be no need to check limits
    return (T)_base_value;
}

#endif

#endif // __LOW_PASS_FILTER_H__

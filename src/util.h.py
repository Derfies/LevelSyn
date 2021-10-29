#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-03-22
#  email:  chongyangm@gmail.com
#  info: math utilities for the template class of a vector
# --------------------------------------------------------------

#ifndef UTIL_H
#define UTIL_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#ifdef WIN32
    #undef min
    #undef max
#endif

using std.max
using std.min
using std.swap

template <class T>
inline T sqr( T& x)
    return x * x


template <class T>
inline T min(T a1, a2, a3)
    return min(a1, min(a2, a3))


template <class T>
inline T min(T a1, a2, a3, a4)
    return min(min(a1, a2), min(a3, a4))


template <class T>
inline T min(T a1, a2, a3, a4, a5)
    return min(min(a1, a2), min(a3, a4), a5)


template <class T>
inline T min(T a1, a2, a3, a4, a5, a6)
    return min(min(a1, a2), min(a3, a4), min(a5, a6))


template <class T>
inline T max(T a1, a2, a3)
    return max(a1, max(a2, a3))


template <class T>
inline T max(T a1, a2, a3, a4)
    return max(max(a1, a2), max(a3, a4))


template <class T>
inline T max(T a1, a2, a3, a4, a5)
    return max(max(a1, a2), max(a3, a4), a5)


template <class T>
inline T max(T a1, a2, a3, a4, a5, a6)
    return max(max(a1, a2), max(a3, a4), max(a5, a6))


template <class T>
inline T clamp(T a, lower, upper)
    if a < lower:
        return lower
    elif a > upper:
        return upper
    else:
        return a


#ifdef WIN32
# there may be some fancy bit-trickery that's faster...
inline long lround(double x)
    if x > 0:
        return (x - floor(x) < 0.5) ? (long)floor(x) : (long)ceil(x)
    else:
        return (x - floor(x) <= 0.5) ? (long)floor(x) : (long)ceil(x)

#endif #WIN32

inline unsigned int round_up_to_power_of_two(unsigned int n)
    exponent = 0
    --n
    while (n)
        ++exponent
        n >>= 1

    return 1 << exponent


inline unsigned int round_down_to_power_of_two(unsigned int n)
    exponent = 0
    while (n > 1)
        ++exponent
        n >>= 1

    return 1 << exponent


inline int intlog2(int x)
    exp = -1
    while (x)
        x >>= 1
        ++exp

    return exp


template <class T>
def set_zero(self, v):
    for (i = (int)v.size() - 1; i >= 0; --i)
        v[i] = 0


template <class T>
def abs_max(self, v):
    m = 0
    for (i = (int)v.size() - 1; i >= 0; --i)
        if std.fabs(v[i]) > m:
            m = std.fabs(v[i])

    return m


template <class T>
def contains(self, a, e):
    for (unsigned i = 0; i < a.size(); ++i)
        if (a[i] == e) return True
    return False


#endif # UTIL_H

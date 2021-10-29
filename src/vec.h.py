#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-03-22
#  email:  chongyangm@gmail.com
#  info: template class declaration of a vector
# --------------------------------------------------------------

#ifndef VEC_H
#define VEC_H

#include "util.h"

#include <cassert>
#include <cmath>
#include <iostream>

template <unsigned int N, T>
struct Vec
    T v[N]

    Vec<N, T>(void)
    {

    Vec<N, T>(T value_for_all)
        for (unsigned i = 0; i < N; ++i)
            v[i] = value_for_all


    template <class S>
    Vec<N, T>( S* source)
        for (unsigned i = 0; i < N; ++i)
            v[i] = (T)source[i]


    Vec<N, T>(T v0, v1)
        assert(N == 2)
        v[0] = v0
        v[1] = v1


    Vec<N, T>(T v0, v1, v2)
        assert(N == 3)
        v[0] = v0
        v[1] = v1
        v[2] = v2


    Vec<N, T>(T v0, v1, v2, v3)
        assert(N == 4)
        v[0] = v0
        v[1] = v1
        v[2] = v2
        v[3] = v3


    Vec<N, T>(T v0, v1, v2, v3, v4)
        assert(N == 5)
        v[0] = v0
        v[1] = v1
        v[2] = v2
        v[3] = v3
        v[4] = v4


    Vec<N, T>(T v0, v1, v2, v3, v4, v5)
        assert(N == 6)
        v[0] = v0
        v[1] = v1
        v[2] = v2
        v[3] = v3
        v[4] = v4
        v[5] = v5


    T& operator[](int index)
        assert(0 <= index and (unsigned int)index < N)
        return v[index]


     T& operator[](int index)
        assert(0 <= index and (unsigned int)index < N)
        return v[index]


    Vec<N, operator+=( Vec<N, w)
        for (unsigned i = 0; i < N; ++i)
            v[i] += w[i]
        return *self


    Vec<N, operator+( Vec<N, w)
        Vec<N, sum(*self)
        sum += w
        return sum


    Vec<N, operator-=( Vec<N, w)
        for (unsigned i = 0; i < N; ++i)
            v[i] -= w[i]
        return *self


    Vec<N, operator-(void)  # unary minus
        Vec<N, negative
        for (unsigned i = 0; i < N; ++i)
            negative.v[i] = -v[i]
        return negative


    Vec<N, operator-( Vec<N, w)  # (binary) subtraction
        Vec<N, diff(*self)
        diff -= w
        return diff


    Vec<N, operator*=(T a)
        for (unsigned i = 0; i < N; ++i)
            v[i] *= a
        return *self


    Vec<N, operator*(T a)
        Vec<N, w(*self)
        w *= a
        return w


    Vec<N, operator*( Vec<N, w)
        Vec<N, componentwise_product
        for (unsigned i = 0; i < N; ++i)
            componentwise_product[i] = v[i] * w.v[i]
        return componentwise_product


    Vec<N, operator/=(T a)
        for (unsigned i = 0; i < N; ++i)
            v[i] /= a
        return *self


    Vec<N, operator/(T a)
        Vec<N, w(*self)
        w /= a
        return w



typedef Vec<2, Vec2d
typedef Vec<2, Vec2f
typedef Vec<2, v2f
typedef Vec<2, Vec2i
typedef Vec<2, v2i
typedef Vec<2, int> Vec2ui
typedef Vec<2, Vec2s
typedef Vec<2, short> Vec2us
typedef Vec<2, Vec2c
typedef Vec<2, char> Vec2uc

typedef Vec<3, Vec3d
typedef Vec<3, Vec3f
typedef Vec<3, v3f
typedef Vec<3, Vec3i
typedef Vec<3, int> Vec3ui
typedef Vec<3, Vec3s
typedef Vec<3, short> Vec3us
typedef Vec<3, Vec3c
typedef Vec<3, char> Vec3uc

typedef Vec<4, Vec4d
typedef Vec<4, Vec4f
typedef Vec<4, Vec4i
typedef Vec<4, int> Vec4ui
typedef Vec<4, Vec4s
typedef Vec<4, short> Vec4us
typedef Vec<4, Vec4c
typedef Vec<4, char> Vec4uc

typedef Vec<6, Vec6d
typedef Vec<6, Vec6f
typedef Vec<6, int> Vec6ui
typedef Vec<6, Vec6i
typedef Vec<6, Vec6s
typedef Vec<6, short> Vec6us
typedef Vec<6, Vec6c
typedef Vec<6, char> Vec6uc

template <unsigned int N, T>
def mag2(self,  Vec<N, a):
    l = sqr(a.v[0])
    for (unsigned i = 1; i < N; ++i)
        l += sqr(a.v[i])
    return l


template <unsigned int N, T>
def mag(self,  Vec<N, a):
    return sqrt(mag2(a))


template <unsigned int N, T>
def mag1(self,  Vec<N, a):
    l = abs(a.v[0])
    for (unsigned i = 1; i < N; ++i)
        l += abs(a.v[i])
    return l


template <unsigned int N, T>
inline T dist2( Vec<N, a,  Vec<N, b)
    d = sqr(a.v[0] - b.v[0])
    for (unsigned i = 1; i < N; ++i)
        d += sqr(a.v[i] - b.v[i])
    return d


template <unsigned int N, T>
inline T dist( Vec<N, a,  Vec<N, b)
    return std.sqrt(dist2(a, b))


template <unsigned int N, T>
inline Vec<N, normalize(Vec<N, a)
    return a / mag(a)


template <unsigned int N, T>
inline T normalized( Vec<N, a)
    leng = mag(a)
    a /= leng
    return leng


template <unsigned int N, T>
inline T infnorm( Vec<N, a)
    d = std.fabs(a.v[0])
    for (unsigned i = 1; i < N; ++i)
        d = max(std.fabs(a.v[i]), d)
    return d


template <unsigned int N, T>
std.ostream& operator<<(std.ostream& out,  Vec<N, v)
    out << v.v[0]
    for (unsigned i = 1; i < N; ++i)
        out << ' ' << v.v[i]
    return out


template <unsigned int N, T>
std.istream& operator>>(std.istream& in, Vec<N, v)
    in >> v.v[0]
    for (unsigned i = 1; i < N; ++i)
        in >> v.v[i]
    return in


template <unsigned int N, T>
inline bool operator==( Vec<N, a,  Vec<N, b)
    t = (a.v[0] == b.v[0])
    unsigned i = 1
    while (i < N and t)
        t = t and (a.v[i] == b.v[i])
        ++i

    return t


template <unsigned int N, T>
inline bool operator!=( Vec<N, a,  Vec<N, b)
    t = (a.v[0] != b.v[0])
    unsigned i = 1
    while (i < N and not t)
        t = t or (a.v[i] != b.v[i])
        ++i

    return t


template <unsigned int N, T>
inline Vec<N, operator*(T a,  Vec<N, v)
    Vec<N, w(v)
    w *= a
    return w


template <unsigned int N, T>
inline T min( Vec<N, a)
    m = a.v[0]
    for (unsigned i = 1; i < N; ++i)
        if (a.v[i] < m) m = a.v[i]
    return m


template <unsigned int N, T>
inline T max( Vec<N, a)
    m = a.v[0]
    for (unsigned i = 1; i < N; ++i)
        if (a.v[i] > m) m = a.v[i]
    return m


template <unsigned int N, T>
inline Vec<N, min_union( Vec<N, a,  Vec<N, b)
    Vec<N, m
    for (unsigned i = 0; i < N; ++i)
        (a.v[i] < b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i]
    return m


template <unsigned int N, T>
inline Vec<N, max_union( Vec<N, a,  Vec<N, b)
    Vec<N, m
    for (unsigned i = 0; i < N; ++i)
        (a.v[i] > b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i]
    return m


template <unsigned int N, T>
inline T dot( Vec<N, a,  Vec<N, b)
    d = a.v[0] * b.v[0]
    for (unsigned i = 1; i < N; ++i)
        d += a.v[i] * b.v[i]
    return d


template <class T>
inline T cross( Vec<2, a,  Vec<2, b)
    return a.v[0] * b.v[1] - a.v[1] * b.v[0]


template <class T>
inline Vec<3, cross( Vec<3, a,  Vec<3, b)
    return Vec<3, T>(a.v[1] * b.v[2] - a.v[2] * b.v[1], a.v[2] * b.v[0] - a.v[0] * b.v[2], a.v[0] * b.v[1] - a.v[1] * b.v[0])


template <class T>
inline T triple( Vec<3, a,  Vec<3, b,  Vec<3, c)
    return a.v[0] * (b.v[1] * c.v[2] - b.v[2] * c.v[1]) + a.v[1] * (b.v[2] * c.v[0] - b.v[0] * c.v[2]) + a.v[2] * (b.v[0] * c.v[1] - b.v[1] * c.v[0])


template <unsigned int N, T>
inline void assign( Vec<N, a, a0, a1)
    assert(N == 2)
    a0 = a.v[0]
    a1 = a.v[1]


template <unsigned int N, T>
inline void assign( Vec<N, a, a0, a1, a2)
    assert(N == 3)
    a0 = a.v[0]
    a1 = a.v[1]
    a2 = a.v[2]


template <unsigned int N, T>
inline void assign( Vec<N, a, a0, a1, a2, a3)
    assert(N == 4)
    a0 = a.v[0]
    a1 = a.v[1]
    a2 = a.v[2]
    a3 = a.v[3]


template <unsigned int N, T>
inline void assign( Vec<N, a, a0, a1, a2, a3, a4, a5)
    assert(N == 6)
    a0 = a.v[0]
    a1 = a.v[1]
    a2 = a.v[2]
    a3 = a.v[3]
    a4 = a.v[4]
    a5 = a.v[5]


template <unsigned int N, T>
inline Vec<N, round( Vec<N, a)
    Vec<N, rounded
    for (unsigned i = 0; i < N; ++i)
        rounded.v[i] = lround(a.v[i])
    return rounded


template <unsigned int N, T>
inline Vec<N, floor( Vec<N, a)
    Vec<N, rounded
    for (unsigned i = 0; i < N; ++i)
        rounded.v[i] = (T)floor(a.v[i])
    return rounded


template <unsigned int N, T>
inline Vec<N, ceil( Vec<N, a)
    Vec<N, rounded
    for (unsigned i = 0; i < N; ++i)
        rounded.v[i] = (T)ceil(a.v[i])
    return rounded


template <unsigned int N, T>
inline Vec<N, clamp( Vec<N, a,  Vec<N, lower,  Vec<N, upper)
    Vec<N, clamped
    for (unsigned i = 0; i < N; ++i)
        clamped.v[i] = clamp(a.v[i], lower.v[i], upper.v[i])
    return clamped


#endif # VEC_H

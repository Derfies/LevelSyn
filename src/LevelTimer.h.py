#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-08-28
#  email:  chongyangm@gmail.com
#  info: class declaration of CLevelTimer for performance measurement
# -------------------------------------------------------------------

#ifndef LEVELTIMER_H
#define LEVELTIMER_H

#ifdef __APPLE__

    #include <CoreFoundation/CoreFoundation.h>

class CLevelTimer
public:
    CLevelTimer() {

    double GetTime()
        time = CFAbsoluteTimeGetCurrent()
        return time


    double GetElapsedTime(double old_time)
        time = CFAbsoluteTimeGetCurrent()
        return (time - old_time)



#endif

#ifdef WIN32

    #include <windows.h>
typedef __int64 i64

class CLevelTimer
public:
    CLevelTimer()
    double GetTime()
    double GetElapsedTime(double old_time)

private:
    i64 m_freq
    i64 m_clocks


CLevelTimer.CLevelTimer() :
    m_clocks(0)
    QueryPerformanceFrequency((LARGE_INTEGER*)&m_freq)


def GetTime(self):
    QueryPerformanceCounter((LARGE_INTEGER*)&m_clocks)
    return (double)m_clocks / (double)m_freq


def GetElapsedTime(self, old_time):
    QueryPerformanceCounter((LARGE_INTEGER*)&m_clocks)
    return ((double)m_clocks / (double)m_freq - old_time)


#endif

#ifdef __linux__

    #include <time.h>

class CLevelTimer
public:
    CLevelTimer() {

    double GetTime()
        time_res = (double)time(NULL)
        return time_res


    double GetElapsedTime(double old_time)
        time_res = time(NULL)
        return (time_res - old_time)



#endif

#endif # LEVELTIMER_H

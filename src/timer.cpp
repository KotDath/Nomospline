#pragma once

#include "timer.h"

Timer::Timer(const QString& startName, const QString& endName) : startName(startName), endName(endName)
{
    m_StartTimepoint = std::chrono::high_resolution_clock::now();
    qDebug() << startName;
}

Timer::~Timer()
{
    if (!isStopped)
    {
        Stop();
    }

}

void Timer::Stop()
{
    auto m_EndTimepoint = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch();
    auto end = std::chrono::time_point_cast<std::chrono::microseconds>(m_EndTimepoint).time_since_epoch();
    auto duration = (end - start).count();
    qDebug() << endName;
    qDebug() << "Total time:  " << duration * 0.001 << " ms";

    isStopped = true;
}

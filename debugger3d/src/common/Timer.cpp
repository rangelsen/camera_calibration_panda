#include <iostream>

#include "Timer.hpp"

TimePoint Timer::tic_start_;

Timer::Timer() { }

Timer::~Timer() { }

////////////////////////////////////////////////////////////////////////////////
void Timer::Start() {

    start_ = std::chrono::system_clock::now();
}

////////////////////////////////////////////////////////////////////////////////
long Timer::ElapsedMillis() const {

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    long elapsed_millis =
    std::chrono::duration_cast<std::chrono::milliseconds> (now - start_).count();

    return elapsed_millis;
}

////////////////////////////////////////////////////////////////////////////////
double Timer::ElapsedSeconds() const {

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::duration<double> dseconds = now - start_;

    return dseconds.count();
}

////////////////////////////////////////////////////////////////////////////////
void Timer::Tic() {

    tic_start_ = std::chrono::system_clock::now();
}

////////////////////////////////////////////////////////////////////////////////
void Timer::Toc() {

    TimePoint now = std::chrono::system_clock::now();

    long elapsed_millis = std::chrono::duration_cast<std::chrono::milliseconds>
        (now - tic_start_).count();

    std::cout << "Timer: Elapsed milliseconds: " << elapsed_millis << std::endl;
}

/// @file


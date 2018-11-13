#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

typedef std::chrono::system_clock::time_point TimePoint;

class Timer {

public:
    Timer();
    ~Timer();

    void Start();

    long ElapsedMillis() const;

    double ElapsedSeconds() const;

    static void Tic();

    /**
     * @brief Display elapsed milliseconds since last call to Toc()
     */
    static void Toc();

private:
    TimePoint start_;
    static TimePoint tic_start_;
};

#endif // TIMER_HPP

/// @file


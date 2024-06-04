#include "../include/rate.h"

Rate::Rate(double frequency) : expected_cycle_time_(std::chrono::duration<double>(1.0 / frequency))
{
    last_time_ = std::chrono::high_resolution_clock::now();
}

void Rate::sleep()
{
    std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - last_time_;
    std::chrono::duration<double> sleep_time = expected_cycle_time_ - elapsed_time;
    if (sleep_time > std::chrono::duration<double>(0))
    {
        std::this_thread::sleep_for(sleep_time);
    }
    else
    {
        // If elapsed time exceeds expected cycle time, do not sleep
    }
    last_time_ = std::chrono::high_resolution_clock::now();
}
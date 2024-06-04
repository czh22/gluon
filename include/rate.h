#ifndef RATE_H_
#define RATE_H_

#include <chrono>
#include <thread>

class Rate {
public:
    Rate(double frequency);

    void sleep();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    std::chrono::duration<double> expected_cycle_time_;
};

#endif // RATE_H_
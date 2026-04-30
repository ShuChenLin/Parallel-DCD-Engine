/**
 * @file timer.h
 * @brief Small wall-clock timer used for stage timing.
 */

#pragma once
#include <chrono>
#include <string>
#include <cstdio>

/**
 * @brief Simple helper for measuring elapsed time in milliseconds.
 */
struct Timer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point start_time;
    std::string label;
    double elapsed_ms = 0;

    void start(const std::string& l = "") {
        label = l;
        start_time = Clock::now();
    }

    double stop() {
        auto end = Clock::now();
        elapsed_ms = std::chrono::duration<double, std::milli>(end - start_time).count();
        return elapsed_ms;
    }

    void print() const {
        printf("  %-25s %8.3f ms\n", label.c_str(), elapsed_ms);
    }
};

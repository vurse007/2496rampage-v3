#pragma once
#include "main.h"
#include "utility.hpp"
#include <vector>
#include <functional>
#include <atomic>
#include <memory>
#include <algorithm>

namespace lynx {

class Queue {
    private:
        struct Action {
            uint32_t execute_time_ms;       // absolute millis when it should run
            std::function<void()> callback; // lambda to run
            bool executed = false;
        };

        std::vector<Action> actions;
        std::atomic<bool> running{false};
        std::unique_ptr<pros::Task> worker;
        lynx::utility::timer auton_timer; // internal timer that starts with scheduler

    public:
        Queue() = default;

        void start();

        void stop();

        // Schedule a relative delay from NOW
        void schedule_delay(uint32_t delay_ms, std::function<void()> func);

        // Schedule a specific timestamp
        void schedule_at(uint32_t target_ms, std::function<void()> func);

        bool is_running() const;

        void clear_executed();

        uint32_t elapsed() const;
    };

} // namespace lynx
#include "lynx/queue.hpp"
#include <cstdint>
namespace lynx{
    void Queue::start(){
        if (running) return;
        running = true;
        auton_timer.restart(); // start internal timer at t=0

        worker = std::make_unique<pros::Task>([this] {
            while (running) {
                uint32_t now = pros::millis();
                for (auto &a : actions) {
                    if (!a.executed && now >= a.execute_time_ms) {
                        a.callback();
                        a.executed = true;
                    }
                }
                pros::delay(10);
            }
        }); 
    }
    
    void Queue::stop(){
        running = false;
        if (worker) worker.reset();
        actions.clear();
    }
    
    void Queue::schedule_delay(uint32_t delay_ms, std::function<void()> func) {
        actions.push_back({
            pros::millis() + delay_ms,
            std::move(func),
            false
        });
    }
    
    void Queue::schedule_at(uint32_t target_ms, std::function<void()> func) {
        uint32_t now = pros::millis();
        uint32_t elapsed = auton_timer.elapsed();
        uint32_t execute_time = now + (target_ms > elapsed ? (target_ms - elapsed) : 0);

        actions.push_back({
            execute_time,
            std::move(func),
            false
        });
    }
    
    void Queue::clear_executed(){
        actions.erase(
            std::remove_if(actions.begin(), actions.end(),
                        [](const Action &a) { return a.executed; }),
            actions.end()
        );
    }
    
    bool Queue::is_running() const{
        return running;
    }
    
    uint32_t Queue::elapsed() const{
        return auton_timer.elapsed();
    }
}
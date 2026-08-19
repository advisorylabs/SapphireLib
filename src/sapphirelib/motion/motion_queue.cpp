#include "sapphirelib/motion/motion_queue.hpp"

#include <utility>

namespace sapphirelib::motion {

void MotionQueue::enqueue(std::function<void()> motion) {
    mutex_.take();
    pending_.push_back(std::move(motion));
    mutex_.give();
}

bool MotionQueue::isBusy() const {
    mutex_.take();
    const bool busy = busy_ || !pending_.empty();
    mutex_.give();
    return busy;
}

void MotionQueue::waitUntilDone() const {
    while (isBusy()) {
        pros::delay(10);
    }
}

void MotionQueue::clear() {
    mutex_.take();
    pending_.clear();
    mutex_.give();
}

void MotionQueue::run() {
    task_ = std::make_unique<pros::Task>(
        [this] {
            while (true) {
                mutex_.take();
                std::function<void()> next;
                if (!pending_.empty()) {
                    next = std::move(pending_.front());
                    pending_.pop_front();
                    busy_ = true;
                } else {
                    busy_ = false;
                }
                mutex_.give();

                if (next) {
                    next();
                } else {
                    pros::delay(10);
                }
            }
        },
        "MotionQueue");
}

} // namespace sapphirelib::motion

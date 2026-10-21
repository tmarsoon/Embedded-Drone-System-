#include "Simulation.h"

#include <stdexcept>

namespace sim {
HubScheduler::HubScheduler(int jobs) {
    if (jobs < 0) throw std::invalid_argument("job count must be nonnegative");
    for (int job = 0; job < jobs; ++job) pending_.push_back(job);
}

std::optional<int> HubScheduler::nextJob() {
    if (pending_.empty()) return std::nullopt;
    const int job = pending_.front();
    pending_.pop_front();
    return job;
}

bool HubScheduler::acquire(int droneId) {
    if (droneId < 0) throw std::invalid_argument("drone ID must be nonnegative");
    if (owner_ && *owner_ != droneId) return false;
    owner_ = droneId;
    return true;
}

void HubScheduler::release(int droneId) {
    if (droneId < 0) throw std::invalid_argument("drone ID must be nonnegative");
    if (!owner_ || *owner_ != droneId) {
        throw std::logic_error("only the corridor owner may release it");
    }
    owner_.reset();
}

std::optional<int> HubScheduler::owner() const { return owner_; }
bool HubScheduler::empty() const { return pending_.empty(); }
} // namespace sim

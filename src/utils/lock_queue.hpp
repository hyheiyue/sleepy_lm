#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace sleepy::utils {

template<typename T>
class LockQueue {
public:
    explicit LockQueue(std::size_t max_size = 0): max_size_(max_size), stop_(false) {}

    bool push(T value) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) {
            return false;
        }
        bool dropped_oldest = false;
        if (max_size_ > 0 && queue_.size() >= max_size_) {
            queue_.pop_front();
            dropped_oldest = true;
        }
        queue_.push_back(std::move(value));
        lock.unlock();
        cv_.notify_one();
        return dropped_oldest;
    }

    bool wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return stop_ || !queue_.empty(); });
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    template<typename Rep, typename Period>
    bool wait_and_pop_for(T& value, const std::chrono::duration<Rep, Period>& timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, timeout, [&] { return stop_ || !queue_.empty(); });
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        T value = std::move(queue_.front());
        queue_.pop_front();
        return value;
    }

    std::vector<T> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return { queue_.begin(), queue_.end() };
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
    }

    bool stopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::deque<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t max_size_;
    bool stop_;
};

} // namespace sleepy::utils

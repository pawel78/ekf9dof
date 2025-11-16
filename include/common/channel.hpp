# pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

namespace channel {

template <typename T>
class Channel {
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    bool closed_ = false;

public:
    // Send data to the channel
    // Returns false if channel is closed, true on success
    bool send(const T& data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) {
                return false;
            }
            queue_.push(data);
        }
        cond_var_.notify_one();
        return true;
    }

    // Receive data from the channel (blocking)
    // Returns std::nullopt if channel is closed and empty
    std::optional<T> receive() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this]() { return !queue_.empty() || closed_; });
        
        if (queue_.empty() && closed_) {
            return std::nullopt;
        }
        
        T data = queue_.front();
        queue_.pop();
        return data;
    }

    // Try to receive data from the channel (non-blocking)
    // Returns false if no data available
    bool try_receive(T& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        data = queue_.front();
        queue_.pop();
        return true;
    }

    // Close the channel (signals no more data will be sent)
    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        cond_var_.notify_all();
    }

    // Check if channel is closed
    bool is_closed() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex_));
        return closed_;
    }

    // Get current queue size (for debugging/monitoring)
    size_t size() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex_));
        return queue_.size();
    }
};

} // namespace channel

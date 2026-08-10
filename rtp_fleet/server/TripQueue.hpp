/**
 * @file TripQueue.hpp
 * @brief Thread-safe queue of completed trips between TCP thread and DB thread.
 *
 * The TCP thread pushes CompletedTrip objects when a trip END is received.
 * The DB thread blocks on pop() until a trip is available, then writes to SQLite.
 * No shmget/shmat needed — both threads are in the same process.
 */

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include "TripManager.hpp"

class TripQueue {
public:
    /** @brief Push a completed trip (called from TCP thread). */
    void push(CompletedTrip trip)
    {
        {
            std::lock_guard<std::mutex> lock(mx_);
            queue_.push(std::move(trip));
        }
        cond_.notify_one();
    }

    /**
     * @brief Pop next trip, blocking until one is available.
     * @param running  Server running flag — stops waiting when false.
     */
    std::optional<CompletedTrip> pop(bool running)
    {
        std::unique_lock<std::mutex> lock(mx_);
        cond_.wait(lock, [&]{ return !queue_.empty() || !running; });

        if (queue_.empty()) return std::nullopt;

        CompletedTrip t = std::move(queue_.front());
        queue_.pop();
        return t;
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mx_);
        return queue_.size();
    }

private:
    mutable std::mutex          mx_;
    std::condition_variable     cond_;
    std::queue<CompletedTrip>   queue_;
};

// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file ThreadPool.hpp
 */

#ifndef __SRC_DDSROUTERCORE_COMMUNICATION_THREADPOOL_HPP_
#define __SRC_DDSROUTERCORE_COMMUNICATION_THREADPOOL_HPP_

#include <communication/DataForwardQueue.hpp>
#include <reader/IReader.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <mutex>
#include <vector>
#include <map>
#include <thread>
#include <chrono>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Thread pool to pull tasks from a DataForwardQueue instance
 */
class ThreadPool
{
public:

    /**
     * @brief Single constructor
     *
     * @param dfq Queue holding data forwarding tasks
     * @param workers_count Number of worker threads
     *
     * @throw InitializationException if number of threads is invalid
     */
    ThreadPool(
            DataForwardQueue& dfq,
            unsigned int workers_count)
        : data_forward_queue_(dfq)
        , count_started_(0)
        , stop_(false)
    {
        if (workers_count == 0)
        {
            throw utils::InitializationException(
                      utils::Formatter() << "Invalid number of worker threads: " << workers_count);
        }

        worker_threads_ = std::vector<std::thread>(workers_count);
    }

    /**
     * @brief Spawn all worker threads
     */
    void start_workers()
    {
        for (auto i = 0u; i < worker_threads_.size(); i++)
        {
            worker_threads_[i] = std::thread( &ThreadPool::do_work_, this, i );
        }

        // Await until all worker threads have started.
        // NOTE: This is a quick implementation. Could be done more cleanly with a CV, but not a priority now.
        while (count_started_.load() < worker_threads_.size())
        {
            logDebug(DDSROUTER_THREAD_POOL, "Awaiting for all worker threads to start " << count_started_.load());
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
        logDebug(DDSROUTER_THREAD_POOL, "All worker threads started successfully ");
    }

    /**
     * @brief Stop all worker threads gathering any eventual exceptions thrown by them
     */
    void stop_workers()
    {

        // Set stop_ to true
        this->stop_.store(true);

        // Then send as many invalid tasks to the queue as workers
        for (auto i = 0u; i < worker_threads_.size(); i++)
        {
            data_forward_queue_.push_task( DataForwardQueue::make_invalid_task());
        }

        logDebug(DDSROUTER_THREAD_POOL, "Joining worker threads");

        // Join workers
        for (auto i = 0u; i < worker_threads_.size(); i++)
        {
            if (worker_threads_[i].joinable())
            {
                // Log joining thread
                logDebug(DDSROUTER_THREAD_POOL, "Joining worker thread " << i);
                worker_threads_[i].join();
            }
            else
            {
                logWarning(DDSROUTER_THREAD_POOL, "Worker thread " << i << " is not joinable");
            }
        }

        logDebug(DDSROUTER_THREAD_POOL, "All worker threads joined");

        if (data_forward_queue_.is_empty())
        {
            logDebug(DDSROUTER_THREAD_POOL, "Task queue is empty after workers joined");
        }
        else
        {
            logDebug(DDSROUTER_THREAD_POOL, "Task queue is not empty after workers joined");
        }

        std::lock_guard<std::mutex> lock(exceptions_mutex_);

        for (const auto& worker_exceptions : all_workers_exceptions_)
        {

            logWarning(DDSROUTER_THREAD_POOL,
                    "Got " << worker_exceptions.second.size() << " exceptions from worker " <<
                    worker_exceptions.first);
            for (const auto exc_ptr : worker_exceptions.second)
            {

                try
                {
                    std::rethrow_exception(exc_ptr);
                }
                catch (const std::exception& exc)
                {
                    logWarning(DDSROUTER_THREAD_POOL,
                            "Worker " << worker_exceptions.first << " exception: " << exc.what());
                }
            }
        }
    }

private:

    /**
     * @brief Function run by worker threads
     *
     * Await for tasks in the queue, and run them
     */
    void do_work_(
            unsigned int worker_id)
    {
        try
        {

            count_started_++;

            while (true)
            {

                auto task = data_forward_queue_.wait_and_pop();

                if (not DataForwardQueue::is_task_invalid(task))
                {

                    task->take_and_forward();

                }
                else if (this->stop_.load())
                {
                    break;
                    logDebug(DDSROUTER_THREAD_POOL, "Worker thread " << worker_id << " terminating after stop request");
                }
                else
                {
                    logDebug(DDSROUTER_THREAD_POOL, "Worker thread " << worker_id << " received invalid task");
                }
            }

        }
        catch (...)
        {
            logWarning(DDSROUTER_THREAD_POOL, "Storing exception of worker thread " << worker_id);

            std::lock_guard<std::mutex> lck(exceptions_mutex_);

            if (all_workers_exceptions_[worker_id].empty())
            {
                all_workers_exceptions_[worker_id] = std::vector<std::exception_ptr>();
            }

            all_workers_exceptions_[worker_id].push_back(std::current_exception());
        }

        count_started_--;

        logDebug(DDSROUTER_THREAD_POOL, "Worker thread terminated");
    }

    ////////
    // MEMBERS

    //! Non-const reference to task queue
    DataForwardQueue& data_forward_queue_;

    //! Number of worker threads that have started
    std::atomic<unsigned> count_started_;

    //! Flag to signal the stop of worker threads
    std::atomic<bool> stop_;

    //! Vector of worker threads
    std::vector<std::thread> worker_threads_;

    //! Mutex protecting exception storage insertion and read
    std::mutex exceptions_mutex_;

    //! Container mapping threads ids to vectors of exceptions thrown by each
    std::map<unsigned int, std::vector<std::exception_ptr>> all_workers_exceptions_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_COMMUNICATION_THREADPOOL_HPP_ */

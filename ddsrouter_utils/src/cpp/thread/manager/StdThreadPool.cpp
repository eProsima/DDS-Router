// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file StdThreadPool.cpp
 *
 */

#include <iostream>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/thread/manager/StdThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

StdThreadPool::StdThreadPool(
        unsigned int n_threads,
        bool start_running /* = true */)
    : task_queue_(0, false)
    , threads_()
    , n_threads_(n_threads)
{
    if (start_running)
    {
        start();
    }
}

StdThreadPool::~StdThreadPool()
{
    stop();
}

void StdThreadPool::start()
{
    if (!task_queue_.enabled())
    {
        logDebug(DDSROUTER_STDTHREADPOOL, "Starting thread pool.");

        task_queue_.enable();

        // Execute all threads
        for (unsigned int i=0; i<n_threads_; i++)
        {
            threads_.push_back(
                CustomThread(
                    std::bind(&StdThreadPool::thread_routine_,
                    this)));
        }
        logDebug(DDSROUTER_STDTHREADPOOL, "Thread Pool started.");
    }
    else
    {
        logInfo(DDSROUTER_STDTHREADPOOL, "Trying to start an already running ThreadPool.");
    }
}

void StdThreadPool::stop()
{
    if (task_queue_.enabled())
    {
        logDebug(DDSROUTER_STDTHREADPOOL, "Stopping thread pool.");

        // Disable task queue so all threads finish
        task_queue_.disable();

        // Wait for all threads
        for (auto& thread : threads_)
        {
            thread.join();
        }
        // Eliminate all threads
        threads_.clear();

        logDebug(DDSROUTER_STDTHREADPOOL, "Thread Pool stopped.");
    }
    else
    {
        logInfo(DDSROUTER_STDTHREADPOOL, "Trying to stop an already running ThreadPool.");
    }
}

void StdThreadPool::execute(std::unique_ptr<ITask>&& task)
{
    task_queue_.produce(std::move(task));
}

void StdThreadPool::thread_routine_()
{
    logDebug(DDSROUTER_STDTHREADPOOL, "Starting thread routine: " << std::this_thread::get_id() << ".");

    try
    {
        while (true)
        {
            logDebug(
                DDSROUTER_STDTHREADPOOL,
                "Thread: " << std::this_thread::get_id() << " free, getting new callback.");

            // Wait till there is a new task available
            auto task = task_queue_.consume();

            logDebug(
                DDSROUTER_STDTHREADPOOL,
                "Thread: " << std::this_thread::get_id() << " executing callback.");

            // Executing callback
            task->operator()();

            // NOTE: at this point task would not be further referenced and it will be destroyed.
        }
    }
    catch (const utils::DisabledException& e)
    {
        logDebug(DDSROUTER_STDTHREADPOOL, "Stopping thread: " << std::this_thread::get_id() << ".");
    }
}

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

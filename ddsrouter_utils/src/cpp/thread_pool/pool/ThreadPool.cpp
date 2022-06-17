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
 * @file ThreadPool.cpp
 *
 * This file contains class ThreadPool implementation.
 */

#include <ddsrouter_utils/exception/DisabledException.hpp>

#include <ddsrouter_utils/thread_pool/pool/ThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

ThreadPool::ThreadPool(
        const uint32_t n_threads)
{
    logDebug(DDSROUTER_THREAD_POOL, "Creating Thread Pool with " << n_threads << " threads.");

    for (uint32_t i = 0; i < n_threads; ++i)
    {
        threads_.emplace_back(
            CustomThread(
                std::bind(&ThreadPool::thread_routine_, this)));
    }

    logDebug(DDSROUTER_THREAD_POOL, "Thread Pool created.");
}

ThreadPool::~ThreadPool()
{
    // Disable queue in case it has not been stopped yet.
    task_queue_.disable();

    for (auto& thread : threads_)
    {
        thread.join();
    }
}

void ThreadPool::emit(
        Task&& task)
{
    task_queue_.produce(std::move(task));
}

void ThreadPool::emit(
        const Task& task)
{
    task_queue_.produce(task);
}

void ThreadPool::thread_routine_()
{
    logDebug(DDSROUTER_THREAD_POOL, "Starting thread routine: " << std::this_thread::get_id() << ".");

    try
    {
        while(true)
        {
            logDebug(DDSROUTER_THREAD_POOL, "Thread: " << std::this_thread::get_id() << " free, getting new callback.");
            Task task = task_queue_.consume();
            logDebug(DDSROUTER_THREAD_POOL, "Thread: " << std::this_thread::get_id() << " executing callback.");
            task();
        }
    }
    catch(const utils::DisabledException& e)
    {
        logDebug(DDSROUTER_THREAD_POOL, "Stopping thread: " << std::this_thread::get_id() << ".");
    }
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

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

#include <ddsrouter_utils/exception/StopException.hpp>

#include <pool/ThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

ThreadPool::ThreadPool(
        std::shared_ptr<TaskQueue> task_queue,
        uint32_t n_threads)
    : task_queue_(task_queue)
{
    for (uint32_t i = 0; i < n_threads; ++i)
    {
        threads_.emplace_back(
            CustomThread(
                std::bind(&ThreadPool::thread_routine_, this)));
    }
}

ThreadPool::~ThreadPool()
{
    // Disable queue in case it has not been stopped yet.
    task_queue_->disable();

    for (auto& thread : threads_)
    {
        thread.join();
    }
}

void ThreadPool::thread_routine_()
{
    logDebug(THREADPOOL_THREAD_ROUTINE, "Starting thread routine: " << std::this_thread::get_id() << ".");

    try
    {
        std::shared_ptr<Task> task = task_queue_->next_task();
        logDebug(THREADPOOL_THREAD_ROUTINE, "Thread: " << std::this_thread::get_id() << " executing callback.");
        (*task)();
    }
    catch(const utils::StopException& e)
    {
        logDebug(THREADPOOL_THREAD_ROUTINE, "Stopping thread: " << std::this_thread::get_id() << ".");
    }
}

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */

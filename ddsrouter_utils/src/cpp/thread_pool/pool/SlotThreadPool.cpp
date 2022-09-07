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
 * @file SlotThreadPool.cpp
 *
 * This file contains class SlotThreadPool implementation.
 */

#include <ddsrouter_utils/exception/ValueNotAllowedException.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_utils/thread_pool/pool/SlotThreadPool.hpp>


namespace eprosima {
namespace ddsrouter {
namespace utils {

SlotThreadPool::SlotThreadPool(
        const uint32_t n_threads)
    : number_of_threads_(n_threads)
    , enabled_(false)
{
    logDebug(DDSROUTER_THREAD_POOL, "Creating Thread Pool with " << n_threads << " threads.");
}

SlotThreadPool::~SlotThreadPool()
{
    disable();
    // Disable queue in case it has not been stopped yet.
    task_queue_.disable();

    for (auto& thread : threads_)
    {
        thread.join();
    }
}

void SlotThreadPool::enable() noexcept
{
    if (!enabled_.exchange(true))
    {
        // Execute threads
        for (uint32_t i = 0; i < number_of_threads_; ++i)
        {
            threads_.emplace_back(
                CustomThread(
                    std::bind(&SlotThreadPool::thread_routine_, this)));
        }
    }
}

void SlotThreadPool::disable() noexcept
{
    if (enabled_.exchange(false))
    {
        // Disable Task Queue, so threads will stop eventually when their current task is finished
        task_queue_.disable();

        for (auto& thread : threads_)
        {
            thread.join();
        }

        threads_.clear();
    }
}

void SlotThreadPool::emit(
        const TaskId& task_id)
{
    // TODO check if we want this check or we trust the user of this class
    // {
    //     // Lock to access the slot map while searching the task exists
    //     std::shared_lock<SlotsMapType> lock(slots_);

    //     auto it = slots_.find(task_id);
    //     if (it == slots_.end())
    //     {
    //         throw utils::ValueNotAllowedException(STR_ENTRY << "Slot " << task_id << " not registered.");
    //     }
    // }

    task_queue_.produce(task_id);
}

void SlotThreadPool::slot(
        const TaskId& task_id,
        Task&& task)
{
    // Lock to access the slot map to modify it
    std::unique_lock<SlotsMapType> lock(slots_);

    auto it = slots_.find(task_id);

    if (it != slots_.end())
    {
        throw utils::ValueNotAllowedException(STR_ENTRY << "Slot " << task_id << " already exists.");
    }
    else
    {
        slots_.insert(std::make_pair(task_id, std::move(task)));
    }
}

void SlotThreadPool::thread_routine_()
{
    logDebug(DDSROUTER_THREAD_POOL, "Starting thread routine: " << std::this_thread::get_id() << ".");

    try
    {
        while (true)
        {
            logDebug(DDSROUTER_THREAD_POOL, "Thread: " << std::this_thread::get_id() << " free, getting new callback.");
            TaskId task_id = task_queue_.consume();

            {
                // Lock to access the slot map while executing task
                // NOTE: this can end before calling task if the task object is copied, but if it is used with
                // a reference, it cannot unlock before calling method
                std::shared_lock<SlotsMapType> lock(slots_);

                auto it = slots_.find(task_id);
                // Check the slot is correct
                if (it == slots_.end())
                {
                    throw utils::ValueNotAllowedException(STR_ENTRY << "Slot " << task_id << " in queue not registered.");
                }

                Task& task = it->second;

                logDebug(DDSROUTER_THREAD_POOL, "Thread: " << std::this_thread::get_id() << " executing callback.");
                task();
            }
        }
    }
    catch (const utils::DisabledException& e)
    {
        logDebug(DDSROUTER_THREAD_POOL, "Stopping thread: " << std::this_thread::get_id() << ".");
    }
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

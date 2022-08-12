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

SlotThreadPool::InternalTaskType::InternalTaskType(
        Task&& task,
        const bool is_reusable)
    : task(std::move(task))
    , is_reusable(is_reusable)
{
}

SlotThreadPool::SlotThreadPool(
        const uint32_t n_threads)
    : number_of_threads_(n_threads)
    , enabled_(false)
{
    logDebug(DDSROUTER_THREAD_POOL, "Creating Thread Pool with " << n_threads << " threads.");

    // Execute threads
    for (uint32_t i = 0; i < number_of_threads_; ++i)
    {
        threads_.emplace_back(
            CustomThread(
                std::bind(&SlotThreadPool::thread_routine_, this)));
    }
}

SlotThreadPool::~SlotThreadPool()
{
    task_queue_.disable();

    for (auto& thread : threads_)
    {
        thread.join();
    }

    threads_.clear();
}

void SlotThreadPool::emit(
        const TaskId& task_id)
{
    {
        // Lock to access the slot map
        std::shared_lock<SlotRegistryType> lock(slots_);

        // Check this Task id exists before sending it to the queue
        auto it = slots_.find(task_id);

        if (it == slots_.end())
        {
            throw utils::ValueNotAllowedException(STR_ENTRY << "Slot " << task_id << " not registered.");
        }
    }
    non_blocking_emit_(task_id);
}

void SlotThreadPool::emit_once(
        Task&& task)
{
    // Get new unique Task id
    TaskId new_task_id = new_unique_task_id();

    // Register the task
    register_slot_(
        new_task_id,
        std::move(task),
        false);

    // Emit it (non blocking as we know it is already in the registry)
    non_blocking_emit_(new_task_id);
}

TaskId SlotThreadPool::register_slot(
        Task&& task)
{
    // Get new Task id
    TaskId new_task_id = new_unique_task_id();

    // Call internal register slot
    register_slot_(new_task_id, std::move(task), true);

    return new_task_id;
}

void SlotThreadPool::non_blocking_emit_(
        const TaskId& task_id)
{
    task_queue_.produce(task_id);
}

void SlotThreadPool::register_slot_(
        const TaskId& task_id,
        Task&& task,
        const bool reusable)
{
    // Lock to access the slot map
    std::unique_lock<SlotRegistryType> lock(slots_);

    auto it = slots_.find(task_id);

    if (it != slots_.end())
    {
        // TSNH
        throw utils::ValueNotAllowedException(STR_ENTRY << "Slot " << task_id << " already exists.");
    }
    else
    {
        slots_.insert(std::make_pair(task_id, InternalTaskType(std::move(task), reusable)));
    }
}

void SlotThreadPool::unregister_slot(
        const TaskId& task_id)
{
    // Lock to access the slot map
    std::unique_lock<SlotRegistryType> lock(slots_);

    auto it = slots_.find(task_id);

    if (it == slots_.end())
    {
        throw utils::ValueNotAllowedException(STR_ENTRY << "Slot " << task_id << " not registered.");
    }
    else
    {
        slots_.erase(it);
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

            // Lock to access the slot map
            // NOTE: is done manually and not in new block with lock lock so task is executed without locking
            slots_.lock_shared();

            auto it = slots_.find(task_id);
            // Check the slot is correct
            if (it == slots_.end())
            {
                // TSNH
                throw utils::ValueNotAllowedException(
                    STR_ENTRY << "Slot " << task_id << " in QueueTask must be stored in slots register.");
            }

            InternalTaskType& task = it->second;

            slots_.unlock_shared();

            logDebug(DDSROUTER_THREAD_POOL, "Thread: " << std::this_thread::get_id() << " executing callback.");
            task.task();

            // If task is not reusable, remove it from map
            if (!task.is_reusable)
            {
                unregister_slot(task_id);
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

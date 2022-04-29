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
 * @file TaskQueue.cpp
 *
 * This file contains class TaskQueue implementation.
 */

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/exception/StopException.hpp>

#include <queue/TaskQueue.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

TaskQueue::TaskQueue()
    : waiter_(0, true)
{
}

TaskQueue::~TaskQueue()
{
    waiter_.blocking_disable();
}

void TaskQueue::disable()
{
    waiter_.blocking_disable();
}

void TaskQueue::add_task(std::shared_ptr<Task> task)
{
    task_queue_.Push(task);
    ++waiter_;
}

std::shared_ptr<Task> TaskQueue::take_task_nts_()
{
    if (task_queue_.Empty())
    {
        if (task_queue_.BothEmpty())
        {
            // This should not happen
            throw utils::InconsistencyException("Taking task from an empty queue.");
        }
        task_queue_.Swap();
    }

    auto task = task_queue_.Front();
    task_queue_.Pop();

    --waiter_;

    return task;
}

std::shared_ptr<Task> TaskQueue::next_task()
{
    // Wait until there is at least one task
    event::AwakeReason reason = waiter_.wait_upper_bound_threshold(0);

    if (reason == event::AwakeReason::DISABLED)
    {
        throw utils::StopException("TaskQueue disabled.");
    }

    // Take the first task
    return take_task_nts_();
}

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */

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
 * @file AsyncManager.cpp
 *
 */

#include <iostream>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/thread/manager/AsyncManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

AsyncManager::~AsyncManager()
{
    logDebug(DDSROUTER_THREAD_ASYNCMANAGER, "Closing Async Manager.");
    clean_threads();
    logDebug(DDSROUTER_THREAD_ASYNCMANAGER, "Async Manager closed.");
}

void AsyncManager::execute(std::unique_ptr<ITask>&& task)
{
    // Lock mutex
    std::unique_lock<TasksCollectionType> lock(tasks_running_);

    // Get reference to task
    ITask* task_reference = task.get();

    // Create and Insert task in new index
    // Being indexed in map the unique ptr will not be erased
    tasks_running_.push_back(
        std::make_pair(
            std::make_unique<CustomThread>(
                [task_reference](){
                    task_reference->operator()();
                }
            ),
            std::move(task)
        )
    );

    logDebug(DDSROUTER_THREAD_ASYNCMANAGER, "New thread executing task.");
}

void AsyncManager::clean_threads()
{
    std::unique_lock<TasksCollectionType> lock(tasks_running_);
    for (auto& task : tasks_running_)
    {
        task.first->join();
        logDebug(DDSROUTER_THREAD_ASYNCMANAGER, "Thread finished, removing task and thread.");
    }
    tasks_running_.clear();
}

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

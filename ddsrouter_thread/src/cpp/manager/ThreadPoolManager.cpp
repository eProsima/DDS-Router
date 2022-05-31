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
 * @file ThreadPoolManager.hpp
 *
 * This file contains class ThreadPoolManager implementation.
 */

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_thread/manager/ThreadPoolManager.hpp>
#include <queue/TaskQueue.hpp>
#include <pool/ThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

ThreadPoolManager::ThreadPoolManager(
        uint32_t n_threads)
    : queue_(std::make_shared<TaskQueue>())
    , pool_(std::make_shared<ThreadPool>(queue_, n_threads))
{
    logDebug(DDSROUTER_THREAD_MANAGER, "Thread Pool Manager created.");
}

ThreadPoolManager::~ThreadPoolManager()
{
    logDebug(DDSROUTER_THREAD_MANAGER, "Thread Pool Manager destroyed.");
}

void ThreadPoolManager::emit(
        Task&& task)
{
    queue_->produce(std::move(task));
}

void ThreadPoolManager::emit(
        const Task& task)
{
    queue_->produce(task);
}

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */

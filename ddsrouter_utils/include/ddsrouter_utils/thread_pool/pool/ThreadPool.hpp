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
 * @file ThreadPool.hpp
 *
 * This file contains class ThreadPool definition.
 */

#ifndef _DDSROUTERTHREAD__SRC_CPP_POOL_THREADPOOL_HPP_
#define _DDSROUTERTHREAD__SRC_CPP_POOL_THREADPOOL_HPP_

#include <vector>

#include <ddsrouter_utils/wait/DBQueueWaitHandler.hpp>

#include <ddsrouter_utils/thread_pool/task/Task.hpp>
#include <ddsrouter_utils/thread_pool/thread/CustomThread.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * TODO
 */
class ThreadPool
{
public:

    ThreadPool(
        const uint32_t n_threads);

    ~ThreadPool();

    void emit(
        Task&& task);

    void emit(
        const Task& task);

protected:

    void thread_routine_();

    event::DBQueueWaitHandler<Task> task_queue_;

    std::vector<CustomThread> threads_;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD__SRC_CPP_POOL_THREADPOOL_HPP_ */

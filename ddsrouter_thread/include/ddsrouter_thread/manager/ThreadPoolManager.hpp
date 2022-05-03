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
 * This file contains class ThreadPoolManager definition.
 */

#ifndef _DDSROUTERTHREAD_MANAGER_THREADPOOLMANAGER_HPP_
#define _DDSROUTERTHREAD_MANAGER_THREADPOOLMANAGER_HPP_

#include <queue/TaskQueue.hpp>
#include <pool/ThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

/**
 * TODO
 */
class ThreadPoolManager
{
public:

    ThreadPoolManager(
        uint32_t n_threads);

    ~ThreadPoolManager();

    void emit(
        Task&& task);

protected:

    std::shared_ptr<TaskQueue> queue_;
    std::shared_ptr<ThreadPool> pool_;

};

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD_MANAGER_THREADPOOLMANAGER_HPP_ */

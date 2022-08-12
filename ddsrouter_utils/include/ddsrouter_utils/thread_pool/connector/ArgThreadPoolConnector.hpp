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
 * @file ArgThreadPoolConnector.hpp
 *
 * This file contains class ArgThreadPoolConnector definition.
 */

#ifndef _DDSROUTERUTILS_THREADPOOL_CONNECTOR_ARGSLOTTHREADPOOLCONNECTOR_HPP_
#define _DDSROUTERUTILS_THREADPOOL_CONNECTOR_ARGSLOTTHREADPOOLCONNECTOR_HPP_

#include <map>
#include <thread>
#include <vector>

#include <ddsrouter_utils/atomic/Atomicable.hpp>
#include <ddsrouter_utils/thread_pool/pool/SlotThreadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename ...Args>
class ArgThreadPoolConnector
{
public:

    ArgThreadPoolConnector(
            std::shared_ptr<SlotThreadPool> thread_pool);

    ~ArgThreadPoolConnector();

    void emit(
            const TaskId& task_id,
            Args... args);

    void slot(
            const TaskId& task_id,
            std::function<void(Args...)> callback);

protected:

    void routine_();

    Atomicable<std::queue<Args...>> args_queue_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_THREADPOOL_CONNECTOR_ARGSLOTTHREADPOOLCONNECTOR_HPP_ */

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
 * @file SlotThreadPool.hpp
 *
 * This file contains class SlotThreadPool definition.
 */

#ifndef _DDSROUTERTHREAD__SRC_CPP_POOL_SLOTTHREADPOOL_HPP_
#define _DDSROUTERTHREAD__SRC_CPP_POOL_SLOTTHREADPOOL_HPP_

#include <thread>
#include <vector>

#include <ddsrouter_event/wait/DBQueueWaitHandler.hpp>

#include <ddsrouter_thread/task/Task.hpp>
#include <ddsrouter_thread/task/TaskId.hpp>
#include <ddsrouter_thread/thread/CustomThread.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

/**
 * TODO
 */
class SlotThreadPool
{
public:

    SlotThreadPool(
        const uint32_t n_threads);

    ~SlotThreadPool();

    void emit(
        const TaskId& task_id);

    void slot(
        const TaskId& task_id,
        Task&& task);

protected:

    void thread_routine_();

    event::DBQueueWaitHandler<TaskId> task_queue_;

    std::map<TaskId, Task> slots_;

    std::vector<CustomThread> threads_;

    std::mutex slots_mutex_;

};

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD__SRC_CPP_POOL_SLOTTHREADPOOL_HPP_ */

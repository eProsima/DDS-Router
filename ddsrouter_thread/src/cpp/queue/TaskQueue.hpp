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
 * @file TaskQueue.hpp
 *
 * This file contains class TaskQueue definition.
 */

#ifndef _DDSROUTERTHREAD__SRC_CPP_QUEUE_TASKQUEUE_HPP_
#define _DDSROUTERTHREAD__SRC_CPP_QUEUE_TASKQUEUE_HPP_

#include <thread>
#include <vector>

#include <fastrtps/utils/DBQueue.h>

#include <ddsrouter_event/wait/CounterWaitHandler.hpp>

#include <task/Task.hpp>

namespace eprosima {
namespace ddsrouter {
namespace thread {

/**
 * TODO
 */
class TaskQueue : public std::thread
{
public:

    TaskQueue();

    ~TaskQueue();

    // TODO add enable and logic associated

    // Disable is required to turn off waiter
    void disable();

    void add_task(std::shared_ptr<Task> task);

    std::shared_ptr<Task> next_task();

protected:

    std::shared_ptr<Task> take_task_nts_();

    fastrtps::DBQueue<std::shared_ptr<Task>> task_queue_;

    event::CounterWaitHandler waiter_;
};

} /* namespace thread */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD__SRC_CPP_QUEUE_TASKQUEUE_HPP_ */

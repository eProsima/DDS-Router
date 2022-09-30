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
 * @file StdThreadPool.hpp
 *
 * This file contains class StdThreadPool definition.
 */

#pragma once

#include <map>
#include <thread>
#include <vector>

#include <ddsrouter_utils/library/library_dll.h>
#include <ddsrouter_utils/thread/manager/IManager.hpp>
#include <ddsrouter_utils/thread/thread/CustomThread.hpp>
#include <ddsrouter_utils/wait/DBQueueWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace thread {

/**
 * TODO
 */
class StdThreadPool : public IManager
{
public:

    StdThreadPool(
        unsigned int n_threads,
        bool start_running = true);

    virtual ~StdThreadPool();

    void start();

    void stop();

    virtual void execute(std::unique_ptr<ITask>&& task) override;

protected:

    void thread_routine_();

    /**
     * @brief Double Queue Wait Handler to store task ids
     *
     * This double queue implement methods \c produce , to add tasks to the queue, and \c consume to wait until any
     * task is available, and return the next task available.
     *
     * It will retrieve tasks in FIFO order.
     * Produce and consume methods are not reciprocally blocking.
     */
    event::DBQueueWaitHandler<std::unique_ptr<ITask>> task_queue_;

    /**
     * @brief Threads container
     *
     * @note \c CustomThread are used instead of \c std::thread so some extra logic could be added to threads
     * in future implementation (e.g. performance info).
     */
    std::vector<CustomThread> threads_;

    const unsigned int n_threads_;

};

} /* namespace thread */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

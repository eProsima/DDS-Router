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
 * This class implements a basic Thread Pool.
 *
 * A Thread Pool is a thread container that is able to execute \c Tasks in parallel by redistributing computational
 * charge between the threads that are available.
 * Every thread awaits in a queue until there is task available. Once a task arrives, one of the threads take it and
 * execute it. The other threads keep waiting for other tasks to arrive. Once a thread has finished executing a task,
 * it returns to the queue and waits for another task.
 *
 * This implementation uses a \c DBQueueWaitHandler , this is a Double Queue Wait Handler. This is an object
 * that allow a thread to wait until elements are produced inside, and store this elements in an internal double
 * queue. It allows to add and remove elements from the queue in a thread safe way and without locking the queue.
 * This queue also allows to consume the tasks in the same order they are added to the queue [FIFO].
 *
 * @note Qt notation is used for this implementation, so \c emit means to add a task to the queue
 *
 * @warning even when tasks are forced to be consumed in a sorted way, following FIFO order, it may happen
 * that a later task is executed before a previous task because of system thread scheduling. Both tasks will be
 * taken by some thread, but the order of the running threads is not guaranteed.
 */
class ThreadPool
{
public:

    /**
     * @brief Construct a new Thread Pool object
     *
     * This creates the internal threads in the pool and make them wait for tasks.
     * Each thread is executed with function \c thread_routine_ .
     *
     * @param n_threads number of threads in the pool
     */
    ThreadPool(
        const uint32_t n_threads);

    /**
     * @brief Destroy the Thread Pool object
     *
     * It disables the queue, what makes the threads to stop to finish their tasks and exit.
     */
    ~ThreadPool();

    /**
     * @brief Add a task to be executed by the threads in the pool [move semantics]
     *
     * This add \c task to the queue, and it will be executed whenever a thread is available.
     *
     * @param task new task to be executed by a thread in the pool
     */
    void emit(
        Task&& task);

    /**
     * @brief Add a task to be executed by the threads in the pool [copy semantics]
     *
     * This add \c task to the queue, and it will be executed whenever a thread is available.
     *
     * @param task new task to be executed by a thread in the pool
     */
    void emit(
        const Task& task);

protected:

    /**
     * @brief This is the function that every thread in the pool executes.
     *
     * This function enters an infinite loop where it \c consume an element <Task> from the queue (this means it will
     * wait for a task to be added to the queue in case it is empty, or it will take one if any available).
     * Once a task is available, it will execute it and afterwards it will return to consume another task.
     * This will be repeated until the queue is disabled, what is communicated by a \c DisabledException .
     */
    void thread_routine_();

    /**
     * @brief Double Queue Wait Handler to store tasks
     *
     * This double queue implement methods \c produce , to add tasks to the queue, and \c consume to wait until any
     * task is available, and return the next task available.
     *
     * It will retrieve tasks in FIFO order.
     * Produce and consume methods are not reciprocally blocking.
     */
    event::DBQueueWaitHandler<Task> task_queue_;

    /**
     * @brief Threads container
     *
     * @note \c CustomThread are used instead of \c std::thread so some extra logic could be added to threads
     * in future implementation (e.g. performance info).
     */
    std::vector<CustomThread> threads_;

};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERTHREAD__SRC_CPP_POOL_THREADPOOL_HPP_ */

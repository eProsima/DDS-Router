// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DataForwardQueue.hpp
 */

#ifndef __SRC_DDSROUTERCORE_COMMUNICATION_DATAFORWARDQUEUE_HPP_
#define __SRC_DDSROUTERCORE_COMMUNICATION_DATAFORWARDQUEUE_HPP_

#include <reader/IReader.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <mutex>
#include <memory>
#include <condition_variable>

namespace eprosima {
namespace ddsrouter {
namespace core {

using DataForwardTask = std::add_pointer_t<IReader>;

/**
 * Thread-safe Multi-Consumer / Multi-Producer task queue.
 */
class DataForwardQueue
{
private:

    //! Node encapsulating a task and a pointer to the next node
    struct Node
    {
        DataForwardTask task;
        std::unique_ptr<Node> next;

        Node()
            : task(make_invalid_task())
        {
        }

        Node(
                DataForwardTask&& t)
            : task(std::move(t))
        {
        }

    };

    std::mutex head_mutex_;

    std::unique_ptr<Node> head_;

    std::mutex tail_mutex_;

    Node* tail_;

    std::condition_variable task_cv_;

    // std::atomic<unsigned> count_tasks_; // Optional

private:

    // methods

    Node* get_tail()
    {
        std::lock_guard<std::mutex> tail_lock(tail_mutex_);
        return tail_;
    }

    //! Returns the node at the head
    std::unique_ptr<Node> pop_head()
    {
        std::unique_ptr<Node> old_head = std::move(head_);
        head_ = std::move(old_head->next);

        // if (old_head) { count_tasks_.fetch_sub(1, std::memory_order_relaxed); } // Optional

        return old_head;
    }

    std::unique_lock<std::mutex> wait_for_task_()
    {

        std::unique_lock<std::mutex> head_lock(head_mutex_);

        task_cv_.wait(head_lock, [&]
                {
                    return head_.get() != get_tail();
                });

        return std::move(head_lock);
    }

    std::unique_ptr<Node> wait_pop_head_()
    {
        std::unique_lock<std::mutex> head_lock(wait_for_task_());
        return pop_head();
    }

public:

    //! Single constructor
    DataForwardQueue()
        : head_(std::make_unique<Node>())
        , tail_(head_.get())
    {
    }

    DataForwardQueue(
            const DataForwardQueue& other) = delete;
    DataForwardQueue& operator =(
            const DataForwardQueue& other) = delete;

    // NOTE: Not currenlty used
    // DataForwardTask try_pop_task()
    // {
    //     auto old_head = pop_head();
    //     return old_head ? old_head->task : make_invalid_task();
    // }

    //! Await until queue is not empty and get a task at the head
    DataForwardTask wait_and_pop()
    {
        logInfo(DDSROUTER_DATA_FORWARD_QUEUE, "Awaiting for forward task ");

        const std::unique_ptr<Node> old_head = wait_pop_head_();

        logInfo(DDSROUTER_DATA_FORWARD_QUEUE, "Pulled task to writer " << old_head->task);


        return old_head->task;
    }

    //! Push task to queue
    void push_task(
            DataForwardTask&& in_task)
    {

        logInfo(DDSROUTER_DATA_FORWARD_QUEUE, "Pushed task to writer " << in_task);

        auto p = std::make_unique<Node>();
        {
            std::lock_guard<std::mutex> tail_lock(tail_mutex_);
            tail_->task = std::move(in_task);
            Node* new_tail = p.get();
            tail_->next = std::move(p);
            tail_ = new_tail;
        }

        // count_tasks_.fetch_add(1, std::memory_order_relaxed); // Optional

        task_cv_.notify_one();
    }

    //! Whether there are tasks or not
    bool is_empty()
    {
        std::lock_guard<std::mutex> head_lock(head_mutex_);
        return (head_.get() == get_tail());
    }

    //! Return an invalid task, used as a poison pill, to shut down worker threads
    static DataForwardTask make_invalid_task() noexcept
    {
        return nullptr;
    }

    //! Predicate checking if a task is valid
    static bool is_task_invalid(
            const DataForwardTask& task) noexcept
    {
        return task == nullptr;
    }

};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_COMMUNICATION_DATAFORWARDQUEUE_HPP_ */

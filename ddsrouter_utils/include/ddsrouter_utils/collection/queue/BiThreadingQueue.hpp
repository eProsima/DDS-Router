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
 * @file BiThreadingQueue.hpp
 */

#ifndef _DDSROUTERUTILS_COLLECTION_QUEUE_NONBLOCKINGQUEUE_HPP
#define _DDSROUTERUTILS_COLLECTION_QUEUE_NONBLOCKINGQUEUE_HPP

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @note This implementation is thread safe as long as push is only done from one thread and pop from one thread.
 * They could be same or different thread, but no more than one per method.
 * In order to implement the complete thread safety, reimplement lock and unlock methods with guards (mutexes).
 */
template <typename T>
class BiThreadingQueue
{
public:

    BiThreadingQueue();

    ~BiThreadingQueue();

    virtual bool push(T* element);

    virtual bool pop(T*& element);

protected:

    virtual void lock_head_();
    virtual void unlock_head_();
    virtual void lock_tail_();
    virtual void unlock_tail_();

    struct Node
    {
        T* element = nullptr;
        Node* next = nullptr;
    };

    Node* head_node_;
    Node* tail_node_;
};

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/collection/queue/impl/BiThreadingQueue.ipp>

#endif /* _DDSROUTERUTILS_COLLECTION_QUEUE_NONBLOCKINGQUEUE_HPP */

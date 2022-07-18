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
 * @file BiThreadingQueue.ipp
 */

#ifndef _DDSROUTERUTILS_COLLECTION_QUEUE_NONBLOCKINGQUEUE_IMPL_IPP
#define _DDSROUTERUTILS_COLLECTION_QUEUE_NONBLOCKINGQUEUE_IMPL_IPP

namespace eprosima {
namespace ddsrouter {
namespace utils {

template <typename T>
BiThreadingQueue<T>::BiThreadingQueue()
{
    head_node_ = new Node(nullptr, nullptr);
    tail_node_ = head_node_;
}

template <typename T>
BiThreadingQueue<T>::~BiThreadingQueue()
{
    // No elements should remain in the queue
    if (head_node_->next != nullptr)
    {
        logError(BITHREADING_QUEUE, "There are elements in a destroying queue.");
    }

    // Remove every element from the queue
    while (head_node_ != nullptr)
    {
        Node* next_node = head_node_->next;
        delete head_node_;
        head_node_ = next_node;
    }
}

template <typename T>
bool BiThreadingQueue<T>::push(T* element)
{
    Node* new_node = new Node(element, nullptr);

    lock_tail_();
    tail_node_.next = new_node;
    tail_node_ = new_node;
    unlock_tail_();

    return true;
}

template <typename T>
bool BiThreadingQueue<T>::pop(T*& element)
{
    lock_head_();

    Node* pop_head = head_node_;

    if (head_node_->next == nullptr)
    {
        unlock_head_();
        return false;
    }

    element = head_node_->next->element;
    head_node_ = head_node_->next;

    unlock_head_();

    delete pop_head;
    return true;
}

template <typename T>
inline void BiThreadingQueue<T>::lock_head_()
{
    // Do nothing
}

template <typename T>
inline void BiThreadingQueue<T>::unlock_head_()
{
    // Do nothing
}

template <typename T>
inline void BiThreadingQueue<T>::lock_tail_()
{
    // Do nothing
}

template <typename T>
inline void BiThreadingQueue<T>::unlock_tail_()
{
    // Do nothing
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_COLLECTION_QUEUE_NONBLOCKINGQUEUE_IMPL_IPP */

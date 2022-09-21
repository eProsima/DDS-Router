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
 * @file DBQueueWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_

#include <fastrtps/utils/DBQueue.h>

#include <ddsrouter_utils/wait/ConsumerWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * This Wait Handler will make threads wait until a data has been added to a Double Queue.
 *
 * The Double Queue works as a queue where pushing data do not block popping data from it, and thus it is a
 * very efficient implementation.
 *
 * \c T specializes this class depending on the data that is stored inside the queue.
 */
template <typename T>
class DBQueueWaitHandler : public ConsumerWaitHandler<T>
{
public:

    // Use parent constructor
    using ConsumerWaitHandler<T>::ConsumerWaitHandler;

protected:

    /**
     * @brief Override of \c ConsumerWaitHandler method to move a new value to the queue
     *
     * @warning \c DBQueue does not allow moving elements, and thus it will always be copied
     *
     * @param value new value to move
     */
    void add_value_(
            T&& value) override;

    /**
     * @brief Override of \c ConsumerWaitHandler method to remove a value from the queue
     *
     * Before removing the value, this method will check if the front queue is empty and swap.
     * This method should not be called if there is no data in the queue.
     *
     * This method is protected with \c pop_queue_mutex so swap cannot be done but from one thread at a time.
     *
     * @throw \c InconsistencyException if it is called without data in the queue
     */
    T get_next_value_() override;

    //! \c DBQueue variable that stores the data
    fastrtps::DBQueue<T> queue_;

    //! Protect getting values from the queue so only one thread can do the swap at a time
    std::mutex pop_queue_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/wait/impl/DBQueueWaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAIT_DBQUEUEWAITHANDLER_HPP_ */

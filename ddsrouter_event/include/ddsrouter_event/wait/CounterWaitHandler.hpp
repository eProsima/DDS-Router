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
 * @file CounterWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_COUNTERWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_COUNTERWAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/library/library_dll.h>
#include <ddsrouter_event/wait/WaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

using CounterType = unsigned int;

/**
 * @brief This WaitHandler class wait threads for a counter value.
 *
 * This class could be understood as: the difference between internal value - threshold is the number of threads
 * that could be awaken.
 *
 * This class holds an internal value that can be incremented or decremented one by one.
 * It also has a constant threshold value.
 * While the internal value overcome the threshold (higher), wait condition would be true for one of the threads.
 * Once a thread has finished waiting, it decrements the internal value in 1.
 *
 * @note This is specialization of \c WaitHandler that is improved for be more efficient then \c IntWaitHandler
 * as it only notify threads when the actually need to wake up, and only one thread at a time.
 * Very useful for consumer wait handlers.
 *
 * @note This class is thread safe and do work properly. Side cases could be:
 * 1. Value is higher than 1+threshold -> the waiting thread that is awaken will
 * decrease value by 1, and then next thread will be notified (one by one)
 * 2. Value is higher than threshold before thread arrive to wait -> predicate is checked
 * at the instantiation time, so it does not need a notify.
 */
class CounterWaitHandler : protected WaitHandler<CounterType>
{
public:

    /**
     * @brief Construct a new Counter Wait Handler object
     *
     * @param threshold value that should be reached (equal or higher) to wake up
     * @param enabled whether the object starts enabled or disabled
     */
    DDSROUTER_EVENT_DllAPI CounterWaitHandler(
            CounterType threshold,
            CounterType initial_value,
            bool enabled = true);

    //! Default constructor
    DDSROUTER_EVENT_DllAPI ~CounterWaitHandler();

    /////
    // Enabling methods

    // Make this methods public
    using WaitHandler<CounterType>::enable;
    using WaitHandler<CounterType>::disable;
    using WaitHandler<CounterType>::blocking_disable;
    using WaitHandler<CounterType>::enabled;
    using WaitHandler<CounterType>::get_value;
    using WaitHandler<CounterType>::stop_and_continue;

    /////
    // Wait methods

    /**
     * @brief Wait current thread while counter does not reach \c threshold and decrease 1 counter in case it does.
     *
     * This is a specialization of this class that allow to decrease -1 to the current counter if thread
     * has been awaken by reaching the threshold.
     *
     * @note Decrease is done only if awaken reason has been \c CONDITION_MET .
     *
     * @param timeout maximum time in milliseconds that should wait until awaking for timeout
     *
     * @return reason why thread was awaken
     */
    DDSROUTER_EVENT_DllAPI AwakeReason wait_and_decrement(
            const utils::Duration_ms& timeout = 0) noexcept;

    /////
    // Value methods

    /**
     * @brief Operator prefix ++ to add 1 to counter
     *
     * It notifies one threads if internal value is higher than threshold.
     *
     * @return this object
     */
    DDSROUTER_EVENT_DllAPI CounterWaitHandler& operator ++();

protected:

    /**
     * @brief Decrease in 1 the internal value and notify threads if is still higher than threshold
     *
     * @warning this method does not lock any mutex. It should be called with \c wait_condition_variable_mutex_ locked.
     */
    void decrease_1_nts_();

    const CounterType threshold_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_COUNTERWAITHANDLER_HPP_ */

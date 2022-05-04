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

#include <ddsrouter_event/wait/WaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

using CounterType = int32_t;

/**
 * @brief This WaitHandler allows to wait for an int value
 *
 * This works as an int value that allows to wait for a specific value or threshold.
 */
class CounterWaitHandler : protected WaitHandler<CounterType>
{
public:

    /**
     * @brief Construct a new Counter Wait Handler object
     *
     * @param value to initialize counter
     * @param enabled whether the object starts enabled or disabled
     */
    CounterWaitHandler(
            CounterType value,
            bool enabled = true);

    //! Default constructor
    ~CounterWaitHandler();

    /////
    // Enabling methods

    // Make this methods public
    using WaitHandler<CounterType>::enable;
    using WaitHandler<CounterType>::disable;
    using WaitHandler<CounterType>::enabled;
    using WaitHandler<CounterType>::set_value;
    using WaitHandler<CounterType>::get_value;
    using WaitHandler<CounterType>::stop_and_continue;

    /////
    // Wait methods

    /**
     * @brief Wait current thread while counter is not \c expected_value .
     *
     * @param expected_value value of counter for which the thread will awake
     * @param timeout maximum time in milliseconds that should wait until awaking for timeout
     *
     * @return reason why thread was awake
     */
    AwakeReason wait_equal(
            CounterType expected_value,
            const utils::Duration_ms& timeout = 0);

    /**
     * @brief Wait current thread while counter is lower or equal than \c upper_bound .
     *
     * @param upper_bound maximum value of counter with which the thread will not awake
     * @param timeout maximum time in milliseconds that should wait until awaking for timeout
     *
     * @return reason why thread was awake
     */
    AwakeReason wait_greater_than(
            CounterType upper_bound,
            const utils::Duration_ms& timeout = 0);

    //! @brief Wait current thread while counter is lower than \c upper_bound .
    AwakeReason wait_greater_equal_than(
            CounterType upper_bound,
            const utils::Duration_ms& timeout = 0);

    /**
     * @brief Wait current thread while counter is higher or equal than \c lower_bound .
     *
     * @param lower_bound minimum value of counter with which the thread will not awake
     * @param timeout maximum time in milliseconds that should wait until awaking for timeout
     *
     * @return reason why thread was awake
     */
    AwakeReason wait_lower_than(
            CounterType lower_bound,
            const utils::Duration_ms& timeout = 0);

    //! Wait current thread while counter is higher than \c lower_bound .
    AwakeReason wait_lower_equal_than(
            CounterType lower_bound,
            const utils::Duration_ms& timeout = 0);

    /////
    // Value methods

    //! Operator prefix ++ to add 1 to counter
    CounterWaitHandler& operator++();

    //! Operator prefix -- to substract 1 to counter
    CounterWaitHandler& operator--();
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_COUNTERWAITHANDLER_HPP_ */

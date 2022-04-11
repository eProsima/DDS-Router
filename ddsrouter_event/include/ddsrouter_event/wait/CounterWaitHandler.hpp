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
 * @file CounterWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_COUNTERWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAITER_COUNTERWAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/wait/VariableWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

using CounterType = uint32_t;

class CounterWaitHandler : protected VariableWaitHandler<CounterType>
{
public:

    CounterWaitHandler(
        bool enabled = true);

    ~CounterWaitHandler();

    /////
    // Enabling methods

    // Make this methods public
    using VariableWaitHandler<CounterType>::enable;
    using VariableWaitHandler<CounterType>::disable;
    using VariableWaitHandler<CounterType>::enabled;

    /////
    // Wait methods

    virtual AwakeReason wait(
            const CounterType& threshold,
            const utils::Duration_ms& timeout = 0);

    virtual AwakeReason blocking_wait(
            const CounterType& threshold,
            const utils::Duration_ms& timeout = 0);

    /////
    // Block methods

    using VariableWaitHandler<CounterType>::block;
    using VariableWaitHandler<CounterType>::unblock;

    /////
    // Value methods

    void increment() noexcept;

    CounterType actual_counter() const noexcept;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAITER_COUNTERWAITHANDLER_HPP_ */

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
 * @file CounterWaiter.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_COUNTERWAITER_HPP_
#define _DDSROUTEREVENT_WAITER_COUNTERWAITER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/waiter/VariableWaiter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

using CounterType = uint32_t;

class CounterWaiter : protected VariableWaiter<CounterType>
{
public:

    CounterWaiter(
        bool enabled = true);

    ~CounterWaiter();

    /////
    // Enabling methods

    // Make this methods public
    using VariableWaiter<CounterType>::enable;
    using VariableWaiter<CounterType>::disable;
    using VariableWaiter<CounterType>::enabled;

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

    using VariableWaiter<CounterType>::block;
    using VariableWaiter<CounterType>::unblock;

    /////
    // Value methods

    void increment() noexcept;

    CounterType actual_counter() const noexcept;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/EventHandler.ipp>

#endif /* _DDSROUTEREVENT_WAITER_COUNTERWAITER_HPP_ */

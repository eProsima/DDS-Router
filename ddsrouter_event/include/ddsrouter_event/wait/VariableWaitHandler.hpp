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
 * @file VariableWaiter.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_VARIABLEWAITER_HPP_
#define _DDSROUTEREVENT_WAITER_VARIABLEWAITER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/waiter/Waiter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename T>
class VariableWaiter : protected Waiter
{
public:

    VariableWaiter(
        bool enabled = true);

    VariableWaiter(
        T init_value,
        bool enabled = true);

    ~VariableWaiter();

    /////
    // Enabling methods

    // Make this methods public
    using Waiter::enable;
    using Waiter::disable;
    using Waiter::enabled;

    /////
    // Wait methods

    virtual AwakeReason wait(
            std::function<bool(T&)> predicate,
            const utils::Duration_ms& timeout = 0);

    virtual AwakeReason blocking_wait(
            std::function<bool(T&)> predicate,
            const utils::Duration_ms& timeout = 0);

    /////
    // Block methods

    void block();

    void unblock();

    /////
    // Value methods

    T get_value() const noexcept;

    void set_value(T new_value) noexcept;

protected:

    AwakeReason wait(
            const utils::Duration_ms& timeout = 0) override;

    std::atomic<T> value_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAITER_VARIABLEWAITER_HPP_ */

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
 * @file BooleanWaiter.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_BOOLEANWAITER_HPP_
#define _DDSROUTEREVENT_WAITER_BOOLEANWAITER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/waiter/VariableWaiter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

class BooleanWaiter : protected VariableWaiter<bool>
{
public:

    BooleanWaiter(
        bool activated = false,
        bool enabled = true);

    ~BooleanWaiter();

    /////
    // Enabling methods

    // Make this methods public
    using VariableWaiter<bool>::enable;
    using VariableWaiter<bool>::disable;
    using VariableWaiter<bool>::enabled;

    /////
    // Wait methods

    virtual AwakeReason wait(
            const utils::Duration_ms& timeout = 0);

    virtual AwakeReason blocking_wait(
            const utils::Duration_ms& timeout = 0);

    /////
    // Block methods

    using VariableWaiter<bool>::block;
    using VariableWaiter<bool>::unblock;

    /////
    // Value methods

    void activate() noexcept;

    void deactivate() noexcept;

    bool active() noexcept;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAITER_BOOLEANWAITER_HPP_ */

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
 * @file BooleanWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_BOOLEANWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAITER_BOOLEANWAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/wait/VariableWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

class BooleanWaitHandler : private VariableWaitHandler<bool>
{
public:

    BooleanWaitHandler(
        bool activated = false,
        bool enabled = true);

    ~BooleanWaitHandler();

    /////
    // Enabling methods

    // Make this methods public
    using VariableWaitHandler<bool>::enable;
    using VariableWaitHandler<bool>::disable;
    using VariableWaitHandler<bool>::enabled;

    /////
    // Wait methods

    AwakeReason wait(
            const utils::Duration_ms& timeout = 0);

    /////
    // Value methods

    void open() noexcept;

    void close() noexcept;

    bool is_open() const noexcept;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAITER_BOOLEANWAITHANDLER_HPP_ */

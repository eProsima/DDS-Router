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
 * @file BooleanWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_BOOLEANWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_BOOLEANWAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/library/library_dll.h>
#include <ddsrouter_event/wait/WaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * @brief This WaitHandler allows to wait for a value to be set to true.
 *
 * This works as a door that could be opened or closed.
 * While open, every thread waiting will be awake, and every new thread will not await.
 * While closed, every thread waiting or new will wait until timeout, disabled, or opened again.
 */
class BooleanWaitHandler : protected WaitHandler<bool>
{
public:

    /**
     * @brief Construct a new Boolean Wait Handler object
     *
     * @param opened whether the object starts opened or closed
     * @param enabled whether the object starts enabled or disabled
     */
    DDSROUTER_EVENT_DllAPI BooleanWaitHandler(
            bool opened = false,
            bool enabled = true);

    //! Default constructor
    DDSROUTER_EVENT_DllAPI ~BooleanWaitHandler();

    /////
    // Enabling methods

    // Make this methods public
    using WaitHandler<bool>::enable;
    using WaitHandler<bool>::disable;
    using WaitHandler<bool>::blocking_disable;
    using WaitHandler<bool>::enabled;
    using WaitHandler<bool>::stop_and_continue;

    /////
    // Wait methods

    /**
     * @brief Wait current thread while object is closed.
     *
     * This thread will be awake when object is opened, when timeout is reached, or when object is disabled.
     *
     * @param timeout maximum time in milliseconds that should wait until awaking for timeout
     * @return reason why thread was awake
     */
    DDSROUTER_EVENT_DllAPI AwakeReason wait(
            const utils::Duration_ms& timeout = 0);

    /////
    // Value methods

    //! Set current status of \c this object as opened (awake every thread)
    DDSROUTER_EVENT_DllAPI void open() noexcept;

    //! Set current status of \c this object as closed (threads must wait)
    DDSROUTER_EVENT_DllAPI void close() noexcept;

    //! Check whether the object is currently opened
    DDSROUTER_EVENT_DllAPI bool is_open() const noexcept;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_BOOLEANWAITHANDLER_HPP_ */

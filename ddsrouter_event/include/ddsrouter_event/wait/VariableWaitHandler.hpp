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
 * @file VariableWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_VARIABLEWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAITER_VARIABLEWAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>

#include <ddsrouter_event/wait/WaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename T>
class VariableWaitHandler
{
public:

    VariableWaitHandler(
        bool enabled = true);

    VariableWaitHandler(
        T init_value,
        bool enabled = true);

    ~VariableWaitHandler();

    /////
    // Enabling methods

    /**
     * @brief Enable object
     *
     * If object is disable, enable it. Otherwise do nothing.
     *
     * @note: A WaitHandler not enabled could not wait.
     */
    virtual void enable() noexcept;

    /**
     * @brief Disable object
     *
     * If object is enable, disable it. Otherwise do nothing.
     * This method does not finished until every waiting thread has finished waiting.
     *
     * @note: A WaitHandler not enabled could not wait.
     */
    virtual void disable() noexcept;

    //! Whether the object is enabled or disabled
    virtual bool enabled() const noexcept;

    /////
    // Wait methods

    AwakeReason wait(
            std::function<bool(const T&)> predicate,
            const utils::Duration_ms& timeout = 0);

    /////
    // Value methods

    T get_value() const noexcept;

    void set_value(T new_value) noexcept;

    /**
     * @brief This thread will wait until every waiting thread has been awaken
     */
    void blocking_awake_all() noexcept;

protected:

    std::atomic<T> value_;

    //! Whether this object is enabled
    std::atomic<bool> enabled_;

    /**
     * @brief Number of threads currently waiting
     */
    std::atomic<uint32_t> threads_waiting_;

    //! Wait condition variable to call waits
    std::condition_variable wait_condition_variable_;

    //! Mutex to protect condition variable and internal variables \c enabled_ \c should_awake_ and \c threads_waiting_
    std::mutex wait_condition_variable_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/wait/impl/VariableWaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAITER_VARIABLEWAITHANDLER_HPP_ */

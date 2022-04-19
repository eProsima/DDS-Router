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
 * @file WaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAITER_WAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAITER_WAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>

#include <ddsrouter_utils/Time.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {


//! Reasons why a thread waiting in a WaitHandler could have been awaken
enum AwakeReason
{
    DISABLED,       //! WaitHandler has been disabled
    TIMEOUT,        //! Timeout set to wait has been reached
    CONDITION_MET,  //! Awake condition has been met
};

/**
 * @brief This object allow to wait multiple threads waiting for other thread to awake them.
 *
 * While enable, every thread that calls \c wait will wait until other thread calls an awake method
 * ( \c awake_one , \c awake_all , \c blocking_awake_all ).
 *
 * Every thread waiting could be awaken by a timeout elapsed, due to calling an awaken method or by disabling Handler.
 *
 * @note This class is useful because it gives an easy API to handle a wait condition variable and every variable that
 * it needs (mutex, stop, predicate, etc.).
 */
template <typename T>
class WaitHandler
{
public:

    WaitHandler(
        bool enabled = true);

    WaitHandler(
        T init_value,
        bool enabled = true);

    ~WaitHandler();

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
     * This method can finish before the rest of threads has finished.
     *
     * @note: A WaitHandler not enabled could not wait.
     */
    virtual void disable() noexcept;

    /**
     * @brief Disable object and wait till every thread has finished
     *
     * If object is enable, disable it. Otherwise do nothing.
     * This method does not finished until every waiting thread has finished waiting.
     *
     * @note: A WaitHandler not enabled could not wait.
     */
    virtual void blocking_disable() noexcept;

    //! Whether the object is enabled or disabled
    virtual bool enabled() const noexcept;

    /////
    // Wait methods

    AwakeReason wait(
            std::function<bool(const T&)> predicate,
            const utils::Duration_ms& timeout = 0);

    /////
    // Value methods

    //! Get current value
    T get_value() const noexcept;

    //! Set new value
    void set_value(T new_value) noexcept;

protected:

    //! Current value
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
#include <ddsrouter_event/wait/impl/WaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAITER_WAITHANDLER_HPP_ */

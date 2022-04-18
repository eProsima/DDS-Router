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
 * @file WaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_WAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_WAITHANDLER_HPP_

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
 * @brief This object allows to make multiple threads wait, until another thread awakes them.
 *
 * While enabled, every thread that calls \c wait will wait until other thread calls an awake method
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

    /**
     * @brief Construct a new Wait Handler object
     *
     * In this case, the variable \c value_ will be initialized by default.
     *
     * @param enabled whether the WaitHandler should be initialized enabled
     */
    WaitHandler(
        bool enabled = true);

    /**
     * @brief Construct a new Wait Handler object
     *
     * @param init_value initial value for the internal value that is checked in wait conditions
     * @param enabled whether the WaitHandler should be initialized enabled
     */
    WaitHandler(
        T init_value,
        bool enabled = true);

    /**
     * @brief Destroy the Wait Handler object
     *
     * It disables and blocks until every thread has finished.
     */
    ~WaitHandler();

    /////
    // Enabling methods

    /**
     * @brief Enable object
     *
     * If object is disabled, enable it. Otherwise do nothing.
     *
     * @note: A WaitHandler not enabled will not wait.
     */
    virtual void enable() noexcept;

    /**
     * @brief Disable object
     *
     * If object is enabled, disable it. Otherwise do nothing.
     * This method may finish before the rest of threads have finished.
     *
     * @note: A disabled WaitHandler  will not wait.
     */
    virtual void disable() noexcept;

    /**
     * @brief Disable object and wait till every thread has finished
     *
     * If object is enabled, disable it. Otherwise do nothing.
     * This method does not finish until every waiting thread has finished waiting.
     */
    virtual void blocking_disable() noexcept;

    //! Whether the object is enabled or disabled
    virtual bool enabled() const noexcept;

    /////
    // Wait methods

    /**
     * @brief Wait the current thread until one of the awaken reasons happen:
     * - DISABLED       : The object has been disabled while this thread was waiting
     * - TIMEOUT        : Timeout has been reached
     * - CONDITION_MET  : Condition set has been fulfilled
     *
     * @param predicate lambda that will be called with internal \c value_ , must return \c true for values
     * that where the thread must awake
     * @param timeout maximum time in milliseconds that should wait until awaking for timeout
     * @return reason why thread was awake
     */
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

    /**
     * @brief  Current value
     *
     * Must be protected with \c status_mutex_ to read and write
     *
     * @note could not be atomic cause it could be a complex type
     */
    T value_;

    //! Whether this object is enabled
    std::atomic<bool> enabled_;

    /**
     * @brief Number of threads currently waiting
     */
    std::atomic<uint32_t> threads_waiting_;

    //! Wait condition variable to call waits
    std::condition_variable wait_condition_variable_;

    //! Mutex to protect condition variable and internal variables \c enabled and \c threads_waiting_
    mutable std::mutex wait_condition_variable_mutex_;

    /**
     * @brief Mutex to protect enable and disable methods
     *
     * Methods \c enable , \c disable and \c blocking_disable ,
     */
    mutable std::recursive_mutex status_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/wait/impl/WaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAIT_WAITHANDLER_HPP_ */

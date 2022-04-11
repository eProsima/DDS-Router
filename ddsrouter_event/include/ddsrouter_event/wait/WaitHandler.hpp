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

#ifndef _DDSROUTEREVENT_WAIT_WAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_WAITHANDLER_HPP_

#include <atomic>
#include <condition_variable>
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
class WaitHandler
{
public:

    WaitHandler(
        bool enabled = true);

    virtual ~WaitHandler();

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

    /**
     * @brief Stop the thread until it is awaken
     *
     * There are three ways to awake a thread waiting in this method:
     * - \c DISABLED WaitHandler has been disabled
     * - \c TIMEOUT Maximum time set has ellapsed
     * - \c CONDITION_MET awake has been called
     *
     * This method handles the \c threads_waiting_ and \c should_awake_ variables
     *
     * @param timeout maximum time to wait in milliseconds. If 0: infinite time. Default: 0.
     *
     * @return AwakeReason Reason why thread was awaken
     */
    AwakeReason wait(
            const utils::Duration_ms& timeout = 0);

    /////
    // Awake methods

    //! Awake all waiting threads
    void awake_all() noexcept;

    /**
     * @brief Awake one of the waiting threads.
     *
     * The thread awaken is chosen random (it uses \c notify_one ).
     *
     * @note If no threads are waiting, do nothing.
     * @note If awaken has been called at least once for each waiting thread, do nothing.
     */
    void awake_one() noexcept;

    /**
     * @brief This thread will wait until every waiting thread has been awaken
     */
    void blocking_awake_all() noexcept;

protected:

    //! Whether this object is enabled
    std::atomic<bool> enabled_;

    /**
     * @brief Number of threads that should be awaken
     *
     * In case awake_one is called, it is incremented by one (unless the current value is same as \c threads_waiting_ )
     * In case awake_all is called, it is set as \c threads_waiting_ .
     *
     * @warning this never be greater than threads_waiting_
     */
    std::atomic<uint32_t> should_awake_;

    /**
     * @brief Number of threads waiting
     *
     */
    std::atomic<uint32_t> threads_waiting_;

    std::condition_variable wait_condition_variable_;

    std::mutex wait_condition_variable_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_WAITHANDLER_HPP_ */

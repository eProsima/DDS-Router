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
 * @file PeriodicEventHandler.hpp
 */

#ifndef _DDSROUTER_EVENT_PERIODICEVENTHANDLER_HPP_
#define _DDSROUTER_EVENT_PERIODICEVENTHANDLER_HPP_

#include <atomic>
#include <functional>
#include <thread>

#include <ddsrouter/types/Time.hpp>
#include <ddsrouter/event/EventHandler.hpp>
#include <ddsrouter/library/ddsrouter_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * It implements the functionality to raise callback periodically with
 * a specific time period.
 *
 * The callback is repeated indefinitely until the object is destroyed.
 */
class DDSROUTER_DllAPI PeriodicEventHandler : public EventHandler<>
{
public:

    /**
     * @brief Construct a new Periodic Event Handler
     *
     * @param period_time : period time in milliseconds for Event to occur. Must be greater than 0.
     *
     * @throw \c InitializationException in case \c period_time is lower than minimum time period (1ms).
     */
    PeriodicEventHandler(
            Duration_ms period_time);

    /**
     * @brief Construct a new Periodic Event Handler with specific callback
     *
     * @param callback : callback to call when period time comes
     * @param period_time : period time in milliseconds for Event to occur. Must be greater than 0.
     *
     * @throw \c InitializationException in case \c period_time is lower than minimum time period (1ms).
     */
    PeriodicEventHandler(
            std::function<void()> callback,
            Duration_ms period_time);

    /**
     * @brief Destroy the PeriodicEventHandler object
     *
     * Calls \c unset_callback
     */
    ~PeriodicEventHandler();

protected:

    /**
     * @brief Internal thread to wait for period and call callback
     *
     * @warning callback is called from this method, so until the
     * callback does not finish, the time will not restart again.
     */
    void period_thread_routine_() noexcept;

    /**
     * @brief Create thread and start period time
     *
     * Only called from \c callback_set_nts_
     *
     * It is already guarded by \c event_mutex_ .
     */
    void start_period_thread_nts_() noexcept;

    /**
     * @brief Stop period time and detach thread
     *
     * Only called from destructor
     *
     * It is already guarded by \c event_mutex_ .
     */
    void stop_period_thread_nts_() noexcept;

    /**
     * @brief Override \c callback_set_ from \c EventHandler .
     *
     * It starts filewatcher if it has not been started.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_set_nts_() noexcept override;

    /**
     * @brief Override \c callback_set_ from \c EventHandler .
     *
     * It stops filewatcher if it has been started.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_unset_nts_() noexcept override;

    //! Period time in milliseconds
    Duration_ms period_time_;

    //! Period thread
    std::thread period_thread_;

    /**
     * @brief Whether the file_watcher has already been started
     *
     * Guarded by \c periodic_wait_mutex_
     */
    std::atomic<bool> timer_active_;

    /**
     * @brief Condition variable to wait until the time has passed or stop when handler disabled
     *
     * Guard by \c periodic_wait_mutex_
     *
     * This condition variable is used to wait for an amount of time, and if the time has passed OR
     * the object has been disabled, it stops.
     */
    mutable std::condition_variable periodic_wait_condition_variable_;

    //! Guard access to \c periodic_wait_condition_variable_
    mutable std::mutex periodic_wait_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_PERIODICEVENTHANDLER_HPP_ */

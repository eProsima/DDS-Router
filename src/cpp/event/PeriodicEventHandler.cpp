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
 * @file PeriodicEventHandler.cpp
 *
 */

#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/event/PeriodicEventHandler.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

PeriodicEventHandler::PeriodicEventHandler(
        Duration_ms period_time)
    : EventHandler<>()
    , period_time_(period_time)
    , timer_active_(false)
{
    // In case period time is set to 0, the object is not created
    if (period_time <= 0)
    {
        throw InitializationException("Periodic Event Handler could no be created with period time 0");
    }

    logDebug(
        DDSROUTER_PERIODICHANDLER,
        "Periodic Event Handler created with period time " << period_time_ << " .");
}

PeriodicEventHandler::PeriodicEventHandler(
        std::function<void()> callback,
        Duration_ms period_time)
    : PeriodicEventHandler(period_time)
{
    set_callback(callback);
}

PeriodicEventHandler::~PeriodicEventHandler()
{
    unset_callback();
}

void PeriodicEventHandler::period_thread_routine_() noexcept
{
    while (timer_active_.load())
    {

        std::unique_lock<std::mutex> lock(periodic_wait_mutex_);

        // Wait for period time or awake if object has been disabled
        wait_condition_variable_.wait_for(
            lock,
            std::chrono::milliseconds(period_time_),
            [this]
            {
                // Exit if number of events is bigger than expected n
                // or if callback is no longer set
                return !timer_active_.load();
            });

        if (!timer_active_.load())
        {
            break;
        }

        event_occurred_();
    }
}

void PeriodicEventHandler::start_period_thread_nts_() noexcept
{
    {
        std::unique_lock<std::mutex> lock(periodic_wait_mutex_);
        timer_active_.store(true);
    }

    period_thread_ = std::thread(
        &PeriodicEventHandler::period_thread_routine_, this);

    logDebug(
        DDSROUTER_PERIODICHANDLER,
        "Periodic Event Handler thread starts with period time " << period_time_ << " .");
}

void PeriodicEventHandler::stop_period_thread_nts_() noexcept
{
    // Set timer as inactive and awake thread so it stops
    {
        std::unique_lock<std::mutex> lock(periodic_wait_mutex_);
        timer_active_.store(false);
    }
    periodic_wait_condition_variable_.notify_one();

    // Wait for the periodic thread to finish as it will skip sleep due to the condition variable
    period_thread_.join();

    logDebug(
        DDSROUTER_PERIODICHANDLER,
        "Periodic Event Handler thread stops.");
}

void PeriodicEventHandler::callback_set_nts_() noexcept
{
    if (!timer_active_)
    {
        start_period_thread_nts_();
    }
}

void PeriodicEventHandler::callback_unset_nts_() noexcept
{
    if (timer_active_)
    {
        stop_period_thread_nts_();
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

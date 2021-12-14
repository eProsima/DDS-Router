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
    , active_(true)
{
    // In case period time is set to 0, the object is not created
    if (period_time <= 0)
    {
        throw InitializationException("Periodic Event Handler could no be created with period time 0");
    }

    start_period_thread_();
}

PeriodicEventHandler::PeriodicEventHandler(
        std::function<void()> callback,
        Duration_ms period_time)
    : EventHandler<>(callback)
    , period_time_(period_time)
    , active_(true)
{
    // In case period time is set to 0, the object is not created
    if (period_time <= 0)
    {
        throw InitializationException("Periodic Event Handler could no be created with period time 0");
    }

    start_period_thread_();
}

PeriodicEventHandler::~PeriodicEventHandler()
{
    stop_period_thread_();
}

void PeriodicEventHandler::period_thread_routine_() noexcept
{
    while (active_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(period_time_));

        // It locks so it is not erased while callback is running
        // This is needed as this is a different thread than the one that will destroy the object
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (!active_.load())
        {
            break;
        }

        event_occurred_();
    }
}

void PeriodicEventHandler::start_period_thread_() noexcept
{
    period_thread_ = std::thread(
        &PeriodicEventHandler::period_thread_routine_, this);
}

void PeriodicEventHandler::stop_period_thread_() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    active_.store(false);
    if (period_thread_.joinable())
    {
        period_thread_.detach();
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

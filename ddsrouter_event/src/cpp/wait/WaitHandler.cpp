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
 * @file FileWatcherHandler.cpp
 *
 */

#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_event/wait/WaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

WaitHandler::WaitHandler(
        bool enabled /* = true */)
    : enabled_(enabled)
    , should_awake_(0)
    , threads_waiting_(0)
{
}

WaitHandler::~WaitHandler()
{
    // Disable object
    disable();
}

void WaitHandler::enable() noexcept
{
    // If disable, enable it. Otherwise do nothing
    if(!enabled_.load())
    {
        // WARNING: enabled_ should be modified with mutex taken
        std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        logDebug(DDSROUTER_WAIT, "Enabling WaitHandler.");
        enabled_.store(true);
    }
    else
    {
        logDebug(DDSROUTER_WAIT, "Enabling not disabled WaitHandler.");
    }
}

void WaitHandler::disable() noexcept
{
    // If enable, disable it. Otherwise do nothing
    if(enabled_.load())
    {
        // WARNING: enabled_ should be modified with mutex taken
        std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        logDebug(DDSROUTER_WAIT, "Disabling WaitHandler.");
        enabled_.store(false);

        blocking_awake_all();
    }
    else
    {
        logDebug(DDSROUTER_WAIT, "Disabling not enabled WaitHandler.");
    }
}

bool WaitHandler::enabled() const noexcept
{
    return enabled_.load();
}

AwakeReason WaitHandler::wait(
        const utils::Duration_ms& timeout /* = 0 */)
{
    // Check if it is disabled and exit
    if (!enabled())
    {
        return AwakeReason::DISABLED;
    }

    // Do wait with mutex taken
    std::unique_lock<std::mutex> lock(wait_condition_variable_mutex_);

    // Increment number of threads waiting
    // WARNING: mutex must be taken
    threads_waiting_++;

    bool finished_for_timeout = false;

    // If timeout is 0, use wait, if not use wait for timeout
    if (timeout > 0)
    {
        finished_for_timeout = wait_condition_variable_.wait_for(
            lock,
            utils::duration_to_ms(timeout),
            [this]
            {
                // Exit if number of events is bigger than expected n
                // or if callback is no longer set
                return should_awake_.load() > 0 || !enabled_.load();
            });
    }
    else
    {
        wait_condition_variable_.wait(
            lock,
            [this]
            {
                // Exit if number of events is bigger than expected n
                // or if callback is no longer set
                return should_awake_.load() > 0 || !enabled_.load();
            });
    }

    // Decrement number of threads that should be awaken
    // NOTE: mutex is still taken
    should_awake_--;

    // Decrement number of threads waiting
    threads_waiting_--;

    // Check awake reason. Mutex is taken so it can not change while checking
    if (!enabled_.load())
    {
        return AwakeReason::DISABLED;
    }
    else if (finished_for_timeout)
    {
        return AwakeReason::TIMEOUT;
    }
    else
    {
        return AwakeReason::CONDITION_MET;
    }
}

void WaitHandler::awake_all() noexcept
{
    // Set number of threads that should be awaken to all
    {
        std::unique_lock<std::mutex> lock(wait_condition_variable_mutex_);
        should_awake_ = threads_waiting_.load();
    }

    // Notify all threads
    wait_condition_variable_.notify_all();
}

void WaitHandler::awake_one() noexcept
{
    // Increment number of threads that should be awaken in 1
    {
        std::unique_lock<std::mutex> lock(wait_condition_variable_mutex_);

        // If every thread waiting should already be awaken, do nothing and finish
        if (threads_waiting_ <= should_awake_)
        {
            return;
        }
        else
        {
            should_awake_++;
        }
    }

    // Notify one thread
    wait_condition_variable_.notify_one();
}

void WaitHandler::blocking_awake_all() noexcept
{
    // NOTE: awake_all is non blocking, so it does not matter calling it with no threads
    // Thus, there is no need to block this method with mutex
    while(threads_waiting_.load() > 0)
    {
        awake_all();
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

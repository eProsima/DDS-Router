// Copyright 2022
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
 * @file WaitHandler.ipp
 */

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/Time.hpp>

#ifndef _DDSROUTEREVENT_WAIT_IMPL_WAITHANDLER_IPP_
#define _DDSROUTEREVENT_WAIT_IMPL_WAITHANDLER_IPP_

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename T>
WaitHandler<T>::WaitHandler(
        bool enabled /* = true */)
    : enabled_(enabled)
    , threads_waiting_(0)
{
}

template <typename T>
WaitHandler<T>::WaitHandler(
        T init_value,
        bool enabled /* = true */)
    : enabled_(enabled)
    , threads_waiting_(0)
    , value_(init_value)
{
}

template <typename T>
WaitHandler<T>::~WaitHandler()
{
    blocking_disable();
}

template <typename T>
void WaitHandler<T>::enable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(status_mutex_);

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
        logDebug(DDSROUTER_WAIT, "Enabling already enabled WaitHandler.");
    }
}

template <typename T>
void WaitHandler<T>::disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(status_mutex_);

    // If enable, disable it. Otherwise do nothing
    if(enabled_.load())
    {
        {
            // WARNING: enabled_ should be modified with mutex taken
            std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
            logDebug(DDSROUTER_WAIT, "Disabling WaitHandler.");
            enabled_.store(false);
        }

        // Do not block for awaken
        wait_condition_variable_.notify_all();
    }
    else
    {
        logDebug(DDSROUTER_WAIT, "Disabling already disabled WaitHandler.");
    }
}

template <typename T>
void WaitHandler<T>::blocking_disable() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(status_mutex_);

    // Disable this object
    disable();

    // Wait till every thread has finished
    while(threads_waiting_.load() > 0)
    {
        // Lock so this thread has no priority over closin threads
        {
            std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        }

        // Notify in case there are still threads waiting to be awaken
        wait_condition_variable_.notify_all();
    }
}

template <typename T>
bool WaitHandler<T>::enabled() const noexcept
{
    return enabled_.load();
}

template <typename T>
AwakeReason WaitHandler<T>::wait(
        std::function<bool(const T&)> predicate,
        const utils::Duration_ms& timeout /* = 0 */)
{
    // Do wait with mutex taken
    std::unique_lock<std::mutex> lock(wait_condition_variable_mutex_);

    // Check if it is disabled and exit
    if (!enabled())
    {
        return AwakeReason::DISABLED;
    }

    // Increment number of threads waiting
    // WARNING: mutex must be taken
    threads_waiting_++;

    bool finished_for_timeout = false;
    utils::Timestamp time_to_wait_until;

    // If timeout is 0, use wait, if not use wait for timeout
    if (timeout > 0)
    {
        time_to_wait_until = utils::now() + utils::duration_to_ms(timeout);
    }
    else
    {
        time_to_wait_until = utils::the_end_of_times();
    }

    finished_for_timeout = wait_condition_variable_.wait_until(
        lock,
        time_to_wait_until,
        [this, predicate]
        {
            // Exit if predicate is true or if this has been disabled
            return !enabled_.load() || predicate(value_);
        });

    // Decrement number of threads waiting
    // NOTE: mutex is still taken
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

template <typename T>
T WaitHandler<T>::get_value() const noexcept
{
    return value_;
}

template <typename T>
void WaitHandler<T>::set_value(T new_value) noexcept
{
    {
        // Mutex must guard the modification of value_
        std::lock_guard<std::mutex> lock(wait_condition_variable_mutex_);
        value_ = new_value;
    }

    wait_condition_variable_.notify_all();
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_WAIT_IMPL_WAITHANDLER_IPP_ */

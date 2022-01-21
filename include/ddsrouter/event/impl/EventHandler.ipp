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
 * @file EventHandler.ipp
 */

#ifndef _DDSROUTER_EVENT_IMPL_EVENTHANDLER_IPP_
#define _DDSROUTER_EVENT_IMPL_EVENTHANDLER_IPP_

#include <functional>

#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename ... Args>
const std::function<void(Args...)> EventHandler<Args...>::DEFAULT_CALLBACK_ =
        [](Args...)
        {
            // This should never happen
            utils::tsnh(
                utils::Formatter() << "This callback must not be called.");
        };

template <typename ... Args>
EventHandler<Args...>::EventHandler()
    : callback_(DEFAULT_CALLBACK_)
    , is_callback_set_(false)
    , number_of_events_registered_(0)
    , threads_waiting_(0)
{
}

template <typename ... Args>
void EventHandler<Args...>::set_callback(
        std::function<void(Args...)> callback) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(event_mutex_);

    {
        // Setting callback
        // Wait mutex must be taken because this variable is used in wait_for_event() wait
        std::lock_guard<std::mutex> lock(wait_mutex_);
        is_callback_set_.store(true);
    }

    callback_ = callback;

    // Call child methods in case they should do something when handler is enabled or change callback
    callback_set_nts_();
}

template <typename ... Args>
void EventHandler<Args...>::unset_callback() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(event_mutex_);

    if (is_callback_set_)
    {
        {
            // Unsetting callback
            // Wait mutex must be taken because this variable is used in wait_for_event() wait
            std::unique_lock<std::mutex> lock(wait_mutex_);
            is_callback_set_.store(false);
        }

        // Awaking every thread waiting in wait_for_event()
        awake_all_waiting_threads_nts_();

        // Setting new callback (even when it will not be called)
        callback_ = DEFAULT_CALLBACK_;

        // Call child methods in case they should do something when handler is disabled
        callback_unset_nts_();
    }
    else
    {
        logWarning(DDSROUTER_HANDLER, "Unsetting callback from an EventHandler that had no callback set.")
    }
}

template <typename ... Args>
bool EventHandler<Args...>::wait_for_event(
        uint32_t n /*= 1*/) const noexcept
{
    if (is_callback_set_.load())
    {
        std::unique_lock<std::mutex> lock(wait_mutex_);

        ++threads_waiting_;

        wait_condition_variable_.wait(
            lock,
            [n, this]
            {
                // Exit if number of events is bigger than expected n
                // or if callback is no longer set
                return number_of_events_registered_ >= n || !is_callback_set_.load();
            });

        --threads_waiting_;
    }

    // Return true if the condition has been fulfilled. It could stop due to an unset callback
    // Note: number_of_events_registered_ does not required a mutex because it is being read and it is atomic
    return number_of_events_registered_.load() >= n;
}

template <typename ... Args>
uint32_t EventHandler<Args...>::event_count() const noexcept
{
    return number_of_events_registered_;
}

template <typename ... Args>
void EventHandler<Args...>::simulate_event_occurred(
        Args... args) noexcept
{
    event_occurred_(args ...);
}

template <typename ... Args>
void EventHandler<Args...>::event_occurred_(
        Args... args) noexcept
{
    // While event occurred is being process, avoid setting/unsetting callback or destroying object
    std::lock_guard<std::recursive_mutex> lock(event_mutex_);

    if (is_callback_set_.load())
    {
        callback_(args ...);

        // TODO: decide if a disabled event should add number of events registered
        {
            // Increase number of callbacks received
            std::lock_guard<std::mutex> lock(wait_mutex_);
            ++number_of_events_registered_;
        }
    }
    else
    {
        logInfo(DDSROUTER_HANDLER, "Skipping callback in a not enabled EventHandler.");
    }

    // Awake every thread waiting for event to occur
    wait_condition_variable_.notify_all();
}

template <typename ... Args>
void EventHandler<Args...>::awake_all_waiting_threads_nts_() noexcept
{
    bool exit = false;

    while (!exit)
    {
        wait_condition_variable_.notify_all();
        {
            std::lock_guard<std::mutex> lock(wait_mutex_);
            exit = threads_waiting_.load() == 0;
        }
    }
}

template <typename ... Args>
void EventHandler<Args...>::callback_set_nts_() noexcept
{
    // Do nothing. Implement it in child classes if needed.
}

template <typename ... Args>
void EventHandler<Args...>::callback_unset_nts_() noexcept
{
    // Do nothing. Implement it in child classes if needed.
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_IMPL_EVENTHANDLER_IPP_ */

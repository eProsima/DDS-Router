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

namespace eprosima {
namespace ddsrouter {
namespace event {

template <typename ... Args>
const std::function<void(Args...)> EventHandler<Args...>::DEFAULT_CALLBACK_ =
        [](Args...)
        {
            logError(DDSROUTER_HANDLER, "This callback should not be called.");
        };

template <typename ... Args>
EventHandler<Args...>::EventHandler()
    : callback_(DEFAULT_CALLBACK_)
    , is_callback_set_(false)
    , number_of_events_registered_(0)
{
}

template <typename ... Args>
EventHandler<Args...>::EventHandler(
        std::function<void(Args...)> callback)
    : callback_(callback)
    , is_callback_set_(true)
    , number_of_events_registered_(0)
{
}

template <typename ... Args>
EventHandler<Args...>::~EventHandler()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
}

template <typename ... Args>
void EventHandler<Args...>::set_callback(
        std::function<void(Args...)> callback) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    is_callback_set_.store(true);
    callback_ = callback;
    callback_set_();
}

template <typename ... Args>
void EventHandler<Args...>::unset_callback() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    is_callback_set_.store(false);
    callback_ = DEFAULT_CALLBACK_;
    callback_unset_();
}

template <typename ... Args>
void EventHandler<Args...>::wait_for_event(
        uint32_t n /*= 1*/) const noexcept
{
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_condition_variable_.wait(
        lock,
        [n, this]
        {
            // Exit if number of events is bigger than expected n
            return number_of_events_registered_ >= n;
        });
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
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    {
        // Lock to avoid changing values while wait is processing condition
        std::lock_guard<std::mutex> lock(wait_mutex_);

        // Call callback
        if (is_callback_set_.load())
        {
            callback_(args ...);
        }
        else
        {
            logWarning(DDSROUTER_HANDLER, "Calling unset callback.");
        }

        // Increase number of callbacks
        ++number_of_events_registered_;
    }

    // Awake every thread waiting for event
    wait_condition_variable_.notify_all();
}

template <typename ... Args>
void EventHandler<Args...>::callback_set_() noexcept
{
    // Do nothing. Implement it in child classes if needed.
}

template <typename ... Args>
void EventHandler<Args...>::callback_unset_() noexcept
{
    // Do nothing. Implement it in child classes if needed.
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_IMPL_EVENTHANDLER_IPP_ */

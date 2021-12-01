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

#ifndef _DDSROUTER_USERINTERFACE_IMPL_HANDLER_IPP_
#define _DDSROUTER_USERINTERFACE_IMPL_HANDLER_IPP_

#include <functional>

#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace ui {

template <class T>
const std::function<void(T)> EventHandler<T>::DEFAULT_CALLBACK_ =
        [](T t)
        {
            logError(DDSROUTER_HANDLER, "This callback should not be called.");
        };

template <class T>
EventHandler<T>::EventHandler()
    : callback_(DEFAULT_CALLBACK_)
    , is_callback_set_(false)
    , number_of_events_registered_(0)
{
}

template <class T>
EventHandler<T>::EventHandler(std::function<void(T)> callback)
    : callback_(callback)
    , is_callback_set_(true)
    , number_of_events_registered_(0)
{
}

template <class T>
void EventHandler<T>::set_callback(
        std::function<void(T)> callback) noexcept
{
    is_callback_set_.store(true);
    callback_ = callback;
    callback_set_();
}

template <class T>
void EventHandler<T>::unset_callback() noexcept
{
    is_callback_set_.store(false);
    callback_ = DEFAULT_CALLBACK_;
    callback_unset_();
}

template <class T>
void EventHandler<T>::wait_for_event(uint32_t n /*= 1*/) const noexcept
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

template <class T>
void EventHandler<T>::force_callback(T arg) noexcept
{
    call_callback_(arg);
}

template <class T>
void EventHandler<T>::call_callback_(T arg) noexcept
{
    {
        // Lock to avoid changing values while wait is processing condition
        std::lock_guard<std::mutex> lock(wait_mutex_);

        // Call callback
        if (is_callback_set_.load())
        {
            callback_(arg);
        }
        else
        {
            logWarning(DDSROUTER_HANDLER, "Calling unset callback with arg: " << arg << ".");
        }

        // Increase number of callbacks
        ++number_of_events_registered_;
    }

    // Awake every thread waiting for event
    wait_condition_variable_.notify_all();
}

template <class T>
void EventHandler<T>::callback_set_() noexcept
{
    // Do nothing. Implement it in child classes if needed.
}

template <class T>
void EventHandler<T>::callback_unset_() noexcept
{
    // Do nothing. Implement it in child classes if needed.
}

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_USERINTERFACE_IMPL_HANDLER_IPP_ */

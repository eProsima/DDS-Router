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
 * @file EventHandler.hpp
 */

#ifndef _DDSROUTER_USERINTERFACE_HANDLER_HPP_
#define _DDSROUTER_USERINTERFACE_HANDLER_HPP_

#include <functional>

namespace eprosima {
namespace ddsrouter {
namespace interface {

/**
 * TODO
 */
template <class T>
class EventHandler
{
public:

    EventHandler();

    EventHandler(std::function<void(T)> callback);

    void set_callback(
            std::function<void(T)> callback) noexcept;

    void unset_callback() noexcept;

    void wait_for_event(uint32_t n = 1) const noexcept;

    void force_callback(T arg) noexcept;

protected:

    void call_callback_(T arg) noexcept;

    virtual void callback_set_() noexcept;

    virtual void callback_unset_() noexcept;

    std::function<void(T)> callback_;

    //! Default callback. It shows a warning that callback is not set
    static const std::function<void(T)> DEFAULT_CALLBACK_;

    std::atomic<uint32_t> number_of_events_registered_;

    std::atomic<bool> is_callback_set_;

    //! Condition variable to wait until the event
    mutable std::condition_variable wait_condition_variable_;

    //! Guard access to \c wait_condition_variable_
    mutable std::mutex wait_mutex_;
};

} /* namespace interface */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/user_interface/impl/EventHandler.ipp>

#endif /* _DDSROUTER_USERINTERFACE_HANDLER_HPP_ */

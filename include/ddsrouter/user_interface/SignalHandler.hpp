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
 * @file SignalHandler.hpp
 */

#ifndef _DDSROUTER_USERINTERFACE_SIGNALHANDLER_HPP_
#define _DDSROUTER_USERINTERFACE_SIGNALHANDLER_HPP_

#include <csignal>
#include <string>
#include <functional>

#include <ddsrouter/user_interface/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace interface {

enum Signals
{
    SIGNAL_SIGINT = SIGINT,
};

/**
 * TODO
 */
template <int SigNum>
class SignalHandler : public EventHandler<int>
{
public:

    SignalHandler() noexcept;

    SignalHandler(std::function<void(int)> callback) noexcept;

    ~SignalHandler();

protected:

    void callback_set_() noexcept override;

    void callback_unset_() noexcept override;

    void add_to_active_handlers_() noexcept;

    void erase_from_active_handlers_() noexcept;

    static void signal_handler_routine_(int signum) noexcept;

    static void set_signal_handler_() noexcept;

    static void unset_signal_handler_() noexcept;

    static std::vector<SignalHandler*> active_handlers_;

    static std::mutex active_handlers_mutex_;
};

} /* namespace interface */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/user_interface/impl/SignalHandler.ipp>

#endif /* _DDSROUTER_USERINTERFACE_SIGNALHANDLER_HPP_ */

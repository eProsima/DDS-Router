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
 * @file SignalHandler.ipp
 *
 */

#ifndef _DDSROUTER_EVENT_IMPL_SIGNALHANDLER_IPP_
#define _DDSROUTER_EVENT_IMPL_SIGNALHANDLER_IPP_

#include <algorithm>
#include <csignal>

#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/event/SignalHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <int SigNum>
std::vector<SignalHandler<SigNum>*> SignalHandler<SigNum>::active_handlers_;

template <int SigNum>
std::mutex SignalHandler<SigNum>::active_handlers_mutex_;

template <int SigNum>
SignalHandler<SigNum>::SignalHandler() noexcept
    : SignalHandler<SigNum>(
        [](int signum)
        {
            logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << signum << " in specific handler .");
        })
{
}

template <int SigNum>
SignalHandler<SigNum>::SignalHandler(
        std::function<void(int)> callback) noexcept
    : EventHandler<int>(callback)
{
    add_to_active_handlers_();
}

template <int SigNum>
SignalHandler<SigNum>::~SignalHandler()
{
    erase_from_active_handlers_();
}

template <int SigNum>
void SignalHandler<SigNum>::callback_set_() noexcept
{
    add_to_active_handlers_();
}

template <int SigNum>
void SignalHandler<SigNum>::callback_unset_() noexcept
{
    erase_from_active_handlers_();
}

template <int SigNum>
void SignalHandler<SigNum>::add_to_active_handlers_() noexcept
{
    std::lock_guard<std::mutex> lock(active_handlers_mutex_);

    logInfo(DDSROUTER_SIGNALHANDLER,
            "Add signal handler to signal " << SigNum << ".");

    active_handlers_.push_back(this);

    // First value included, set signal handler
    if (active_handlers_.size() == 1)
    {
        set_signal_handler_();
    }
}

template <int SigNum>
void SignalHandler<SigNum>::erase_from_active_handlers_() noexcept
{
    std::lock_guard<std::mutex> lock(active_handlers_mutex_);

    logInfo(DDSROUTER_SIGNALHANDLER,
            "Erase signal handler from signal " << SigNum << ".");

    std::remove(
        active_handlers_.begin(),
        active_handlers_.end(),
        this);

    // Last handler erased, unset signal handler
    if (active_handlers_.size() == 0)
    {
        unset_signal_handler_();
    }
}

template <int SigNum>
void SignalHandler<SigNum>::signal_handler_routine_(
        int signum) noexcept
{
    std::lock_guard<std::mutex> lock(active_handlers_mutex_);

    logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << signum << ".");

    if (signum != SigNum)
    {
        logError(DDSROUTER_SIGNALHANDLER, "Signal handler associated to incorrect signal,");
        // Does not raise an exception as it is from signal thread and we dont know how that works.
        // TODO: decide if process should end when ThisShouldNotHappen happens.
        return;
    }

    for (auto it : active_handlers_)
    {
        it->event_occurred_(signum);
    }
}

template <int SigNum>
void SignalHandler<SigNum>::set_signal_handler_() noexcept
{
    logInfo(DDSROUTER_SIGNALHANDLER,
            "Set signal handler for signal " << SigNum << ".");
    signal(SigNum, signal_handler_routine_);
}

template <int SigNum>
void SignalHandler<SigNum>::unset_signal_handler_() noexcept
{
    logInfo(DDSROUTER_SIGNALHANDLER,
            "Unset signal handler for signal " << SigNum << ".");
    signal(SigNum, SIG_DFL);
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_IMPL_SIGNALHANDLER_IPP_ */

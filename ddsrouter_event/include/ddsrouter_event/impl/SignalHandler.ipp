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

#ifndef _DDSROUTEREVENT_IMPL_SIGNALHANDLER_IPP_
#define _DDSROUTEREVENT_IMPL_SIGNALHANDLER_IPP_

#include <algorithm>

#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <int SigNum>
std::vector<SignalHandler<SigNum>*> SignalHandler<SigNum>::active_handlers_;

template <int SigNum>
std::recursive_mutex SignalHandler<SigNum>::active_handlers_mutex_;

template <int SigNum>
std::thread SignalHandler<SigNum>::signal_handler_thread_;

template <int SigNum>
std::atomic<bool> SignalHandler<SigNum>::signal_handler_active_(false);

template <int SigNum>
std::atomic<bool> SignalHandler<SigNum>::signal_received_(false);

template <int SigNum>
std::condition_variable SignalHandler<SigNum>::signal_received_cv_;

template <int SigNum>
std::mutex SignalHandler<SigNum>::signal_received_cv_mutex_;

template <int SigNum>
SignalHandler<SigNum>::SignalHandler() noexcept
    : SignalHandler<SigNum>(
        [](int signum)
        {
            logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << signum << " in specific handler.");
        })
{
}

template <int SigNum>
SignalHandler<SigNum>::SignalHandler(
        std::function<void(int)> callback) noexcept
    : EventHandler<int>()
{
    set_callback(callback);
}

template <int SigNum>
SignalHandler<SigNum>::~SignalHandler()
{
    std::lock_guard<std::recursive_mutex> lock(active_handlers_mutex_);
    unset_callback();
    logDebug(DDSROUTER_SIGNALHANDLER, "SignalHandler destroyed for signal: " << SigNum << ".");
}

template <int SigNum>
void SignalHandler<SigNum>::callback_set_nts_() noexcept
{
    add_to_active_handlers_();
}

template <int SigNum>
void SignalHandler<SigNum>::callback_unset_nts_() noexcept
{
    erase_from_active_handlers_();
}

template <int SigNum>
void SignalHandler<SigNum>::add_to_active_handlers_() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(active_handlers_mutex_);

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
    std::lock_guard<std::recursive_mutex> lock(active_handlers_mutex_);

    logInfo(DDSROUTER_SIGNALHANDLER,
            "Erase signal handler from signal " << SigNum << ".");

    active_handlers_.erase(std::remove(active_handlers_.begin(), active_handlers_.end(), this));

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
    std::lock_guard<std::recursive_mutex> lock(active_handlers_mutex_);

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
void SignalHandler<SigNum>::signal_handler_thread_routine_() noexcept
{
    signal(SigNum, [](int)
            {
                signal_received_.store(true);
                signal_received_cv_.notify_one();
            });

    signal_handler_active_.store(true);
    while (true)
    {
        std::unique_lock<std::mutex> lock(signal_received_cv_mutex_);
        signal_received_cv_.wait(
            lock,
            []
            {
                return signal_received_.load() || !signal_handler_active_.load();
            });

        if (signal_received_.load())
        {
            signal_received_.store(false);
            signal_handler_routine_(SigNum);
        }
        else
        {
            break;
        }
    }
}

template <int SigNum>
void SignalHandler<SigNum>::set_signal_handler_() noexcept
{
    logInfo(DDSROUTER_SIGNALHANDLER,
            "Set signal handler for signal " << SigNum << ".");

    signal_handler_thread_ = std::thread(signal_handler_thread_routine_);
}

template <int SigNum>
void SignalHandler<SigNum>::unset_signal_handler_() noexcept
{
    logInfo(DDSROUTER_SIGNALHANDLER,
            "Unset signal handler for signal " << SigNum << ".");

    signal_handler_active_.store(false);

    signal_received_cv_.notify_one();

    signal_handler_thread_.join();
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTEREVENT_IMPL_SIGNALHANDLER_IPP_ */

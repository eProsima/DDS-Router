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
 * @file SignalManager.ipp
 *
 */

#ifndef __SRC_DDSROUTEREVENT_IMPL_SIGNALMANAGER_IPP_
#define __SRC_DDSROUTEREVENT_IMPL_SIGNALMANAGER_IPP_

#include <algorithm>

#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <int SigNum>
SignalManager<SigNum>& SignalManager<SigNum>::get_instance() noexcept
{
    static SignalManager<SigNum> instance_;
    return instance_;
}

template <int SigNum>
SignalManager<SigNum>::SignalManager() noexcept
    : signal_handler_thread_stop_(false)
    , signals_received_(0)
{
    signal(SigNum, [](int)
        {
            SignalManager<SigNum>::get_instance().signal_received_();
        });

    logDebug(DDSROUTER_SIGNALMANAGER,
            "Set SignalManager handling signal: " << SigNum << ".");

    signal_handler_thread_ = std::thread(
        &SignalManager<SigNum>::signal_handler_thread_routine_, this);
}

template <int SigNum>
SignalManager<SigNum>::~SignalManager() noexcept
{
    {
        std::unique_lock<std::mutex> lock(signal_received_cv_mutex_);
        signal_handler_thread_stop_.store(true);
    }

    signal_received_cv_.notify_all();
    signal_handler_thread_.join();

    logDebug(DDSROUTER_SIGNALMANAGER,
            "Destroying SignalManager in signal: " << SigNum << ".");
}

template <int SigNum>
UniqueCallbackId SignalManager<SigNum>::add_callback(std::function<void()> callback) noexcept
{
    std::lock_guard<std::mutex> lock(active_callbacks_mutex_);

    UniqueCallbackId new_id = new_unique_id_();
    active_callbacks_[new_id] = callback;

    logDebug(DDSROUTER_SIGNALMANAGER,
            "Add callback to signal " << SigNum << ".");

    return new_id;
}

template <int SigNum>
void SignalManager<SigNum>::erase_callback(UniqueCallbackId id) noexcept
{
    std::lock_guard<std::mutex> lock(active_callbacks_mutex_);

    active_callbacks_.erase(id);

    logDebug(DDSROUTER_SIGNALHANDLER,
            "Erase callback from signal " << SigNum << ".");
}

template <int SigNum>
UniqueCallbackId SignalManager<SigNum>::new_unique_id_() noexcept
{
    std::lock_guard<std::mutex> lock(last_id_mutex_);
    current_last_id_++;
    return current_last_id_;
}

template <int SigNum>
void SignalManager<SigNum>::signal_received_() noexcept
{
    // Normally \c signals_received_ should be guarded by \c signal_received_cv_mutex_ in order to prevent
    // missing notifications. However here it is not possible as mutexes should not be taken from within
    // signal handlers (e.g. the signal may be captured from the same thread holding the mutex causing a
    // deadlock).
    signals_received_++;
    signal_received_cv_.notify_one();
}

template <int SigNum>
void SignalManager<SigNum>::signal_handler_routine_() noexcept
{
    std::lock_guard<std::mutex> lock(active_callbacks_mutex_);

    logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << SigNum << ".");

    for (auto it : active_callbacks_)
    {
        it.second();
    }
}

template <int SigNum>
void SignalManager<SigNum>::signal_handler_thread_routine_() noexcept
{
    while (!signal_handler_thread_stop_.load())
    {
        std::unique_lock<std::mutex> lock(signal_received_cv_mutex_);
        signal_received_cv_.wait(
            lock,
            [this]
            {
                return signals_received_.load() > 0 ||
                    signal_handler_thread_stop_.load();
            });

        if (signal_handler_thread_stop_.load())
        {
            break;
        }
        else if (signals_received_.load() > 0)
        {
            signals_received_--;
            signal_handler_routine_();
        }
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTEREVENT_IMPL_SIGNALMANAGER_IPP_ */

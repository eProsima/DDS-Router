// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

template <Signal SigVal>
std::recursive_mutex SignalManager<SigVal>::instance_mutex_;

template <Signal SigVal>
std::condition_variable SignalManager<SigVal>::signal_received_cv_;

template <Signal SigVal>
std::atomic<uint32_t> SignalManager<SigVal>::signals_received_(0);

template <Signal SigVal>
SignalManager<SigVal>& SignalManager<SigVal>::get_instance() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(instance_mutex_);

    static SignalManager<SigVal> instance_;
    return instance_;
}

template <Signal SigVal>
SignalManager<SigVal>::SignalManager() noexcept
    : signal_handler_thread_stop_(false)
    , current_last_id_(0)
{
    signal(static_cast<SignalType>(SigVal), SignalManager<SigVal>::signal_handler_function_);

    logDebug(DDSROUTER_SIGNALMANAGER,
            "Set SignalManager handling signal: " << SigVal << ".");

    signal_handler_thread_ = std::thread(
        &SignalManager<SigVal>::signal_handler_thread_routine_, this);
}

template <Signal SigVal>
SignalManager<SigVal>::~SignalManager() noexcept
{
    {
        std::unique_lock<std::mutex> lock(signal_received_cv_mutex_);
        signal_handler_thread_stop_.store(true);
    }

    signal_received_cv_.notify_all();
    signal_handler_thread_.join();

    logDebug(DDSROUTER_SIGNALMANAGER,
            "Destroying SignalManager in signal: " << SigVal << ".");
}

template <Signal SigVal>
UniqueCallbackId SignalManager<SigVal>::register_callback(
        std::function<void()> callback) noexcept
{
    std::lock_guard<std::mutex> lock(active_callbacks_mutex_);

    UniqueCallbackId new_id = new_unique_id_();
    active_callbacks_[new_id] = callback;

    logDebug(DDSROUTER_SIGNALMANAGER,
            "Add callback to signal " << SigVal << ".");

    return new_id;
}

template <Signal SigVal>
void SignalManager<SigVal>::unregister_callback(
        UniqueCallbackId id)
{
    std::lock_guard<std::mutex> lock(active_callbacks_mutex_);

    if (!active_callbacks_.erase(id))
    {
        throw utils::InconsistencyException("Unregistering callback that has not been registered before.");
    }

    logDebug(DDSROUTER_SIGNALHANDLER,
            "Erase callback from signal " << SigVal << ".");
}

template <Signal SigVal>
UniqueCallbackId SignalManager<SigVal>::new_unique_id_() noexcept
{
    std::lock_guard<std::mutex> lock(last_id_mutex_);
    current_last_id_++;
    return current_last_id_;
}

template <Signal SigVal>
void SignalManager<SigVal>::signal_handler_function_(
        SignalType sigval) noexcept
{
#ifdef _WIN32
    // Windows requires to handle again the signal once it has been handled
    signal(SigVal, SignalManager<SigVal>::signal_handler_function_);
#endif // _WIN32
    signal_received_();
}

template <Signal SigVal>
void SignalManager<SigVal>::signal_received_() noexcept
{
    // Normally \c signals_received_ should be guarded by \c signal_received_cv_mutex_ in order to prevent
    // missing notifications. However here it is not possible as mutexes should not be taken from within
    // signal handlers (e.g. the signal may be captured from the same thread holding the mutex causing a
    // deadlock).
    signals_received_++;
    signal_received_cv_.notify_one();
}

template <Signal SigVal>
void SignalManager<SigVal>::signal_handler_routine_() noexcept
{
    std::lock_guard<std::mutex> lock(active_callbacks_mutex_);

    logInfo(DDSROUTER_SIGNALHANDLER,
            "Received signal " << SigVal << ".");

    for (auto it : active_callbacks_)
    {
        it.second();
    }
}

template <Signal SigVal>
void SignalManager<SigVal>::signal_handler_thread_routine_() noexcept
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

std::ostream& operator <<(
        std::ostream& os,
        const Signal& sigval) {
    os << static_cast<eprosima::ddsrouter::event::SignalType>(sigval);
    return os;
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTEREVENT_IMPL_SIGNALMANAGER_IPP_ */

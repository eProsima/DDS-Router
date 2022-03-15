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
 * @file SignalManager.hpp
 */

#ifndef __SRC_DDSROUTEREVENT_SIGNALMANAGER_HPP_
#define __SRC_DDSROUTEREVENT_SIGNALMANAGER_HPP_

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace eprosima {
namespace ddsrouter {
namespace event {

//! Available data types for SignalManager class
enum Signals
{
    SIGNAL_SIGINT   = SIGINT,   //! SIGINT  = ^C    = 2
    SIGNAL_SIGTERM  = SIGTERM,  //! SIGTERM = kill  = 15
    SIGNAL_SIGUSR1  = SIGUSR1,  //! SIGUSR1 =       = 10
};

using UniqueCallbackId = uint32_t;

/**
 * TODO
 */
template <int SigNum>
class SignalManager
{
public:

    static SignalManager& get_instance() noexcept;

    UniqueCallbackId add_callback(
        std::function<void()> callback) noexcept;

    void erase_callback(UniqueCallbackId id) noexcept;

    SignalManager(SignalManager const&) = delete;
    void operator=(SignalManager const&) = delete;

protected:

    SignalManager() noexcept;

    ~SignalManager() noexcept;

    UniqueCallbackId new_unique_id_() noexcept;

    void signal_received_() noexcept;

    void signal_handler_routine_() noexcept;

    void signal_handler_thread_routine_() noexcept;


    std::map<UniqueCallbackId, std::function<void()>> active_callbacks_;

    std::mutex active_callbacks_mutex_;


    std::thread signal_handler_thread_;

    std::atomic<bool> signal_handler_thread_stop_;

    //! Counter incremented when a signal arrives and decremented after being read by \c signal_handler_thread_
    std::atomic<uint32_t> signals_received_;

    //! Condition variable to wait in \c signal_handler_thread_ until a signal arrives or signal handler is unset
    std::condition_variable signal_received_cv_;

    //! Guards access to \c signal_received_cv_
    std::mutex signal_received_cv_mutex_;

    std::mutex last_id_mutex_;

    UniqueCallbackId current_last_id_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/SignalManager.ipp>

#endif /* __SRC_DDSROUTEREVENT_SIGNALMANAGER_HPP_ */

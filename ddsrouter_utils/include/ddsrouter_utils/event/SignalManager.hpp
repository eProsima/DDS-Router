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

using SignalType = int;

//! Available data types for SignalManager class
enum class Signal : SignalType
{
    sigint   = SIGINT,   //! SIGINT  = ^C    = 2
    sigterm  = SIGTERM,  //! SIGTERM = kill  = 15

#ifndef _WIN32
    sigusr1  = SIGUSR1,  //! SIGUSR1 =       = 10
    sigusr2  = SIGUSR2,  //! SIGUSR2 =       = 12
#endif // ifndef _WIN32

};

using UniqueCallbackId = uint32_t;

/**
 * This class creates a Singleton object that manages a specific signal given by template specialization \c SigVal .
 *
 * Signals may be captured from any running thread, but handling is performed here in a dedicated thread
 * \c signal_handler_thread_ in order to avoid unexpected behaviors.
 *
 * Callbacks could be registered in this object. Which each signal received every callback registered is called.
 *
 * In creation, this object set the \c signal method to handle signal to call an internal method that augments
 * signals arrived and awake the signal thread.
 * Signal thread is an internal thread (created in constructor) that is awake every time a signal arrives or in
 * destruction.
 * This thread calls every callback registered in this singleton.
 *
 * This class is a template so every signal handled have its own singleton object.
 * Be aware that objects for different signals are completely independent as they are different objects
 * that only share the same templatization.
 *
 * @note this class must be exported as public because SignalEventHandler is a template and it requires to use it,
 * so as the implementation of SignalEventHandler must be public, this too. But it is not mean to be used externally.
 *
 */
template <Signal SigVal>
class SignalManager
{
public:

    //! Get Singleton object
    static SignalManager& get_instance() noexcept;

    /**
     * @brief Add callback that will be called every time a signal arrives
     *
     * A callback must be unregistered before the object that registers it is destroyed.
     * Thus, must be a way to recognize every callback registered.
     * This is done by a unique id that is retrieved by \c this with every callback registered, and this id
     * must be used to unregister this callback.
     *
     * @param callback new callback
     * @return UniqueCallbackId id which this callback will be registered
     */
    UniqueCallbackId register_callback(
            std::function<void()> callback) noexcept;

    /**
     * @brief Unregister a callback by its id
     *
     * @param id id of the callback to unregister
     *
     * @throw \c InconsistencyException in case the callback was not registered
     */
    void unregister_callback(
            UniqueCallbackId id);

    //! Deleted copy method
    SignalManager(
            SignalManager const&) = delete;
    //! Deleted copy method
    void operator =(
            SignalManager const&) = delete;

protected:

    /**
     * @brief Private constructor
     *
     * Set the \c signal function to awake \c signal_handler_thread_ .
     * Create the \c signal_handler_thread_  to call callbacks in signal receive.
     */
    SignalManager() noexcept;

    /**
     * @brief Destroy singleton
     *
     * This method would only be called at the end of the process.
     *
     * It will awake \c signal_handler_thread_  and wait for it to be joined
     */
    ~SignalManager() noexcept;

    //! Give a new unique id for a new callback
    UniqueCallbackId new_unique_id_() noexcept;

    //! function called from \c signal when a signal is handled.
    void signal_handler_routine_() noexcept;

    /**
     * @brief Routine for \c signal_handler_thread_
     *
     * It is an infinite loop that waits for a signal to come or till \c signal_handler_thread_stop_ is set to stop.
     * If awaken by signal received, it calls \c signal_handler_routine_ once.
     * If more than one signal received, it will enter in wait again and exit by the predicate, without waiting.
     */
    void signal_handler_thread_routine_() noexcept;

    /**
     * @brief Method called every time the signal arrives
     *
     * It increments \c signals_received_ by one and awakes \c signal_handler_thread_
     */
    static void signal_received_() noexcept;

    //! Static function to call from \c signal . It only calls singleton \c signal_received_ .
    static void signal_handler_function_(
            int sigval) noexcept;

    //////
    // Callbacks registered
    /**
     * @brief Map of callbacks registered indexed by their unique ids given when registered
     *
     * Guarded by \c active_callbacks_mutex_
     */
    std::map<UniqueCallbackId, std::function<void()>> active_callbacks_;

    //! Guards access to read and writer \c active_callbacks_
    std::mutex active_callbacks_mutex_;

    //////
    // Internal signal thread
    /**
     * @brief Internal thread that awakes from a wait with every signal or in destruction.
     *
     */
    std::thread signal_handler_thread_;

    //! Whether \c signal_handler_thread_ must stop. Only set to true in destruction.
    std::atomic<bool> signal_handler_thread_stop_;

    //! Guards access to \c signal_received_cv_
    std::mutex signal_received_cv_mutex_;

    /**
     * @brief Counter incremented when a signal arrives and decremented after being read by \c signal_handler_thread_.
     *
     * Although in principle it is not required to be static (as it belongs to a singleton class), it is made so to
     * avoid race conditions when a signal is caught, thus interrumpting the execution of a thread which may be locking
     * internal resources required by \c get_instance.
     */
    static std::atomic<uint32_t> signals_received_;

    //! Condition variable to wait in \c signal_handler_thread_ until a signal arrives or signal handler is unset.
    static std::condition_variable signal_received_cv_;

    //////
    // Unique id
    //! Guards access to variable \c current_last_id_
    std::mutex last_id_mutex_;

    //! Variable to store the last unique id registered, so the new one is unique.
    UniqueCallbackId current_last_id_;

    //! Guards access to singleton instance
    static std::recursive_mutex instance_mutex_;
};

std::ostream& operator <<(
        std::ostream& os,
        const Signal& sigval);

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/event/impl/SignalManager.ipp>

#endif /* __SRC_DDSROUTEREVENT_SIGNALMANAGER_HPP_ */

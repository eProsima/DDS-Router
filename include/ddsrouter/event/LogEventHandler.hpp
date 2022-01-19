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
 * @file LogEventHandler.hpp
 */

#ifndef _DDSROUTER_EVENT_LOGEVENTHANDLER_HPP_
#define _DDSROUTER_EVENT_LOGEVENTHANDLER_HPP_

#include <atomic>
#include <functional>

#include <ddsrouter/event/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * It implements the functionality to raise callback every time a Log msg is consumed.
 *
 * @warning Fast DDS Log takes the ownership of the pointer of every new consumer (because of reasons...)
 * Thus, in order to create this kind of handler, it must be created from a pointer (new) and the ownership
 * of the pointer will be lost as soon as it is created.
 */
class LogEventHandler : public EventHandler<fastdds::dds::Log::Entry>, fastdds::dds::LogConsumer
{
public:

    // This class does not have constructor without callback.
    // This is because of the lost of the pointer once it is registered in Fast. This makes it simpler.

    /**
     * Construct a Log Event Handler with callback and enable it.
     *
     * Calls \c set_callback
     *
     * @param callback callback to call every time a log entry is consumed.
     */
    LogEventHandler(
            std::function<void(eprosima::fastdds::dds::Log::Entry)> callback);

    /**
     * @brief Destroy the LogEventHandler object
     *
     * Calls \c unset_callback
     */
    ~LogEventHandler();

    void Consume(
            const Log::Entry& entry) override;

protected:

    /**
     * @brief Override \c callback_set_nts_ from \c EventHandler .
     *
     * The first time is called, it registers the consumer into Log.
     */
    virtual void callback_set_nts_() noexcept override;

    //! Whether the object has been registered in the Log (as soon as it is created in this class)
    std::atomic<bool> first_registered_;

    /**
     * @brief  Vector of Log entries consumed so far.
     *
     * Guarded by \c entries_mutex_ .
     */
    std::vector<Log::Entry> entries_consumed_;

    //! Guard access to \c entries_consumed_
    mutable std::mutex entries_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_LOGEVENTHANDLER_HPP_ */

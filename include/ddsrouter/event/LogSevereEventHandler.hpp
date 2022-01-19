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

#ifndef _DDSROUTER_EVENT_LOGSEVEREEVENTHANDLER_HPP_
#define _DDSROUTER_EVENT_LOGSEVEREEVENTHANDLER_HPP_

#include <functional>

#include <ddsrouter/event/LogEventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * @brief This class implements a LogEventHandler object that only consumes logs that are above a threshold.
 *
 * This is useful to only consume those logs that are Warning and/or Errors.
 */
class LogSevereEventHandler : public LogEventHandler
{
public:

    /**
     * Construct a Severe Log Event Handler with callback and enable it, setting a minimum threshold.
     *
     * Uses \c LogEventHandler constructor.
     *
     * @param callback callback to call every time a log entry is consumed.
     * @param threshold minimum log kind that will be consumed.
     */
    LogSevereEventHandler(
            std::function<void(eprosima::fastdds::dds::Log::Entry)> callback,
            const Log::Kind threshold = Log::Kind::Warning);

    //! Override parent \c Consume method but only consuming logs above the \c threshold_ kind.
    void Consume(
            const Log::Entry& entry) override;

protected:

    //! Minimum Log kind accepted to consumed.
    Log::Kind threshold_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_EVENT_LOGSEVEREEVENTHANDLER_HPP_ */

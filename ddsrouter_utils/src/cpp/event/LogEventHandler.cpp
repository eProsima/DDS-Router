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
 * @file LogEventHandler.cpp
 *
 */

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/event/LogEventHandler.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

LogEventHandler::LogEventHandler(
        std::function<void(utils::Log::Entry)> callback)
    : EventHandler<utils::Log::Entry>()
    , first_registered_(false)
{
    set_callback(callback);
}

LogEventHandler::~LogEventHandler()
{
    unset_callback();
}

void LogEventHandler::callback_set_nts_() noexcept
{
    if (!first_registered_)
    {
        utils::Log::RegisterConsumer(std::unique_ptr<utils::LogConsumer>(this));
        logDebug(
            DDSROUTER_LOGEVENTHANDLER,
            "Log Event Handler Register as Log Consumer (user lost its pointer ownership).");
    }
    first_registered_ = true;
}

void LogEventHandler::Consume(
        const utils::Log::Entry& entry)
{
    {
        std::lock_guard<std::mutex> lock(entries_mutex_);
        entries_consumed_.push_back(entry);
    }

    event_occurred_(entry);
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

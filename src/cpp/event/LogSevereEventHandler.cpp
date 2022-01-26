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
 * @file LogSevereEventHandler.cpp
 *
 */

#include <ddsrouter/event/LogSevereEventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

LogSevereEventHandler::LogSevereEventHandler(
        std::function<void(eprosima::fastdds::dds::Log::Entry)> callback,
        Log::Kind threshold /* = Log::Kind::Warning */)
    : LogEventHandler(callback)
    , threshold_(threshold)
{
    // If verbosity is currently lower than needed, increase it
    if (fastdds::dds::Log::GetVerbosity() < threshold_)
    {
        fastdds::dds::Log::SetVerbosity(threshold_);
    }
}

void LogSevereEventHandler::Consume(
        const Log::Entry& entry)
{
    if (entry.kind <= threshold_)
    {
        LogEventHandler::Consume(entry);
    }
}

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

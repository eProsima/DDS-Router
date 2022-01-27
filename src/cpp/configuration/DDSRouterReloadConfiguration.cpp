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
 * @file DDSRouterReloadConfiguration.cpp
 *
 */

#include <ddsrouter/configuration/DDSRouterReloadConfiguration.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace configuration {

DDSRouterReloadConfiguration::DDSRouterReloadConfiguration(
        std::set<std::shared_ptr<FilterTopic>> allowlist,
        std::set<std::shared_ptr<FilterTopic>> blocklist,
        std::set<std::shared_ptr<RealTopic>> builtin_topics)
    : allowlist_(allowlist)
    , blocklist_(blocklist)
    , builtin_topics_(builtin_topics)
{
}

std::set<std::shared_ptr<FilterTopic>> DDSRouterReloadConfiguration::allowlist() const noexcept
{
    return allowlist_;
}

std::set<std::shared_ptr<FilterTopic>> DDSRouterReloadConfiguration::blocklist() const noexcept
{
    return blocklist_;
}

std::set<std::shared_ptr<RealTopic>> DDSRouterReloadConfiguration::builtin_topics() const noexcept
{
    return builtin_topics_;
}

bool DDSRouterReloadConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check Allow list topics
    for (std::shared_ptr<FilterTopic> topic : allowlist_)
    {
        if (!topic)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in allowlist topics.");
            error_msg << "nullptr Filter Topic in allowlist. ";
            return false;
        }

        if (!topic->is_valid())
        {
            error_msg << "Invalid Filter Topic " << topic << " in allowlist. ";
            return false;
        }
    }

    // Check Block list topics
    for (std::shared_ptr<FilterTopic> topic : blocklist_)
    {
        if (!topic)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in blocklist topics.");
            error_msg << "nullptr Filter Topic in blocklist. ";
            return false;
        }

        if (!topic->is_valid())
        {
            error_msg << "Invalid Filter Topic " << topic << " in blocklist. ";
            return false;
        }
    }

    // Check Builtin list topics
    for (std::shared_ptr<RealTopic> topic : builtin_topics_)
    {
        if (!topic)
        {
            logError(DDSROUTER_CONFIGURATION, "Invalid ptr in builtin topics.");
            error_msg << "nullptr Topic in builtin. ";
            return false;
        }

        if (!topic->is_valid())
        {
            error_msg << "Invalid Topic " << topic << " in Builtin list. ";
            return false;
        }
    }

    return true;
}

} /* namespace configuration */
} /* namespace ddsrouter */
} /* namespace eprosima */

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

#include <ddsrouter_core/configuration/DDSRouterReloadConfiguration.hpp>
#include <cpp_utils/Log.hpp>
#include <ddsrouter_core/types/topic/filter/WildcardDdsFilterTopic.hpp>
#include <cpp_utils/exception/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

using namespace eprosima::ddsrouter::core::types;

DDSRouterReloadConfiguration::DDSRouterReloadConfiguration(
        std::set<std::shared_ptr<DdsFilterTopic>> allowlist,
        std::set<std::shared_ptr<DdsFilterTopic>> blocklist,
        std::set<std::shared_ptr<DdsTopic>> builtin_topics)
    : allowlist(allowlist)
    , blocklist(blocklist)
    , builtin_topics(builtin_topics)
{
}

bool DDSRouterReloadConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check Allow list topics
    for (std::shared_ptr<DdsFilterTopic> topic : allowlist)
    {
        if (!topic)
        {
            logDevError(DDSROUTER_CONFIGURATION, "Invalid ptr in allowlist topics.");
            error_msg << "nullptr Filter Topic in allowlist. ";
            return false;
        }
    }

    // Check Block list topics
    for (std::shared_ptr<DdsFilterTopic> topic : blocklist)
    {
        if (!topic)
        {
            logDevError(DDSROUTER_CONFIGURATION, "Invalid ptr in blocklist topics.");
            error_msg << "nullptr Filter Topic in blocklist. ";
            return false;
        }
    }

    // Check Builtin list topics
    for (std::shared_ptr<DdsTopic> topic : builtin_topics)
    {
        if (!topic)
        {
            logDevError(DDSROUTER_CONFIGURATION, "Invalid ptr in builtin topics.");
            error_msg << "nullptr Topic in builtin. ";
            return false;
        }

        if (!topic->is_valid(error_msg))
        {
            error_msg << "Invalid Topic " << topic << " in Builtin list. ";
            return false;
        }
    }

    return true;
}

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

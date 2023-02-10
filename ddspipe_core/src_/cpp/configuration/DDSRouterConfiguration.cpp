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
 * @file DDSRouterConfiguration.cpp
 *
 */

#include <ddspipe_core/configuration/DDSRouterConfiguration.hpp>
#include <cpp_utils/Log.hpp>

#include <cpp_utils/exception/ConfigurationException.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

using namespace eprosima::ddsrouter::core::types;

DDSRouterConfiguration::DDSRouterConfiguration(
        const std::set<std::shared_ptr<DdsFilterTopic>>& allowlist,
        const std::set<std::shared_ptr<DdsFilterTopic>>& blocklist,
        const std::set<std::shared_ptr<DistributedTopic>>& builtin_topics,
        const SpecsConfiguration& advanced_options)
    : DDSRouterReloadConfiguration (allowlist, blocklist, builtin_topics)
    , advanced_options(advanced_options)
{
}

bool DDSRouterConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    // Check Allow list topics
    if (!DDSRouterReloadConfiguration::is_valid(error_msg))
    {
        return false;
    }

    return true;
}

void DDSRouterConfiguration::reload(
        const DDSRouterReloadConfiguration& new_configuration)
{
    this->allowlist = new_configuration.allowlist;
    this->blocklist = new_configuration.blocklist;
    this->builtin_topics = new_configuration.builtin_topics;
}

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

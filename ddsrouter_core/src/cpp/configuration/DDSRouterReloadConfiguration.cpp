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
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_core/types/topic/Topic.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace configuration {

using namespace eprosima::ddsrouter::core::types;

DDSRouterReloadConfiguration::DDSRouterReloadConfiguration(
        TopicKeySet<FilterTopic> allowlist,
        TopicKeySet<FilterTopic> blocklist,
        TopicKeySet<RealTopic> builtin_topics)
    : allowlist_(allowlist)
    , blocklist_(blocklist)
    , builtin_topics_(builtin_topics)
{
}

const TopicKeySet<FilterTopic>& DDSRouterReloadConfiguration::allowlist() const noexcept
{
    return allowlist_;
}

const TopicKeySet<FilterTopic>& DDSRouterReloadConfiguration::blocklist() const noexcept
{
    return blocklist_;
}

const TopicKeySet<RealTopic>& DDSRouterReloadConfiguration::builtin_topics() const noexcept
{
    return builtin_topics_;
}

TopicKeySet<RealTopic> DDSRouterReloadConfiguration::reload(
        const DDSRouterReloadConfiguration& new_configuration)
{
    // Check if there are any new builtin topics
    TopicKeySet<RealTopic> new_builtin_topics;
    for (const auto& builtin_topic : builtin_topics_)
    {
        if (builtin_topics_.find(builtin_topic) == builtin_topics_.end())
        {
            new_builtin_topics.insert(builtin_topic);
        }
    }

    if (new_builtin_topics.empty() && (new_configuration.allowlist() ==
            this->allowlist_) && (new_configuration.blocklist() == this->blocklist_))
    {
        logDebug(DDSROUTER, "Same configuration, do nothing in reload.");
        return new_builtin_topics;
    }

    allowlist_ = new_configuration.allowlist();
    blocklist_ = new_configuration.blocklist();

    logDebug(DDSROUTER, "New DDS Router allowed topics configuration.");

    return new_builtin_topics;
}

bool DDSRouterReloadConfiguration::register_topic(
        const RealTopic& topic)
{
    if (std::find(std::begin(builtin_topics_), std::end(builtin_topics_), topic) == std::end(builtin_topics_))
    {
        builtin_topics_.insert(topic);
        return true;
    }
    return false;
}

bool DDSRouterReloadConfiguration::is_topic_registered(
        const RealTopic& topic) const noexcept
{
    return std::find(std::begin(builtin_topics_), std::end(builtin_topics_), topic) != std::end(builtin_topics_);
}

bool DDSRouterReloadConfiguration::is_topic_allowed(
        const RealTopic& topic) const noexcept
{
    bool accepted = allowlist_.empty();

    // Check if allowlist filter it (this will do anything if empty and accepted will be true)
    for (const auto& filter : allowlist_)
    {
        if (filter.matches(topic))
        {
            accepted = true;
            break;
        }
    }

    // Check if it has not passed the allowlist so blocklist is skipped
    if (!accepted)
    {
        return false;
    }

    // Allowlist passed, check blocklist
    for (const auto& filter : blocklist_)
    {
        if (filter.matches(topic))
        {
            return false;
        }
    }

    // Blocklist passed, the topic is allowed
    return true;
}

std::ostream& operator <<(
        std::ostream& os,
        const DDSRouterReloadConfiguration& cfg)
{
    os << "AllowedTopicList{";

    // Allowed topics
    os << "allowed(";
    for (const auto& filter_topic : cfg.allowlist())
    {
        os << filter_topic << ",";
    }
    os << ")";

    // Blocked topics
    os << "blocked(";
    for (const auto& filter_topic : cfg.blocklist())
    {
        os << filter_topic << ",";
    }
    os << ")";

    // Builtin topics
    os << "builtin_topics(";
    for (const auto& topic : cfg.builtin_topics())
    {
        os << topic << ",";
    }
    os << ")";

    os << "}";
    return os;
}

} /* namespace configuration */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

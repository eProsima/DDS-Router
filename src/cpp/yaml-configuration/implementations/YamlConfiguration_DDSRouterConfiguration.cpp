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
 * @file YamlConfiguration_DDSRouterConfiguration.cpp
 *
 */

#include <ddsrouter/exception/ConfigurationException.hpp>
#include <ddsrouter/exception/UnsupportedException.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

std::set<std::shared_ptr<FilterTopic>> YamlDDSRouterConfiguration::allowlist(const Yaml& yaml)
{
    return generic_filter_topic(yaml, ALLOWLIST_TAG);
}

std::set<std::shared_ptr<FilterTopic>> YamlDDSRouterConfiguration::blocklist(const Yaml& yaml)
{
    return generic_filter_topic(yaml, BLOCKLIST_TAG);
}

std::set<std::shared_ptr<RealTopic>> YamlDDSRouterConfiguration::builtin_topics(const Yaml& yaml)
{
    throw UnsupportedException("builtin_topics");
}

std::set<std::shared_ptr<FilterTopic>> YamlDDSRouterConfiguration::generic_filter_topic(
    const Yaml& yaml,
    std::string list_tag)
{
    std::set<std::shared_ptr<FilterTopic>> result;

    if (!yaml.IsSequence() && !yaml.IsNull())
    {
        throw ConfigurationException(utils::Formatter() << list_tag << " expects an array or an empty yaml.");
    }

    if (yaml[list_tag])
    {
        for (auto topic : yaml[list_tag])
        {
            result.insert(YamlElementConfiguration::filter_topic(topic));
        }
    }

    return result;
}

std::set<std::shared_ptr<configuration::ParticipantConfiguration>> YamlDDSRouterConfiguration::participants_configurations(const Yaml& yaml)
{
    std::set<std::shared_ptr<configuration::ParticipantConfiguration>> result;

    if (yaml[PARTICIPANTS_LIST_TAG])
    {
        for (auto participant_yaml : yaml[PARTICIPANTS_LIST_TAG])
        {
            result.insert(YamlParticipantConfiguration::participant_configuration_factory(participant_yaml));
        }
    }
    else
    {
        // TODO
        throw ConfigurationException("participants_configurations");
    }

    return result;
}


configuration::DDSRouterConfiguration YamlDDSRouterConfiguration::ddsrouter_configuration(const Yaml& yaml)
{
    std::set<std::shared_ptr<FilterTopic>> allowlist_ = allowlist(yaml);
    std::set<std::shared_ptr<FilterTopic>> blocklist_ = blocklist(yaml);
    std::set<std::shared_ptr<RealTopic>> builtin_topics_ = builtin_topics(yaml);
    std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participants_configurations_ = participants_configurations(yaml);

    return configuration::DDSRouterConfiguration(allowlist_, blocklist_, builtin_topics_, participants_configurations_);
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

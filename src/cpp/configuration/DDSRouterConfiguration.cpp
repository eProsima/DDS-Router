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
 * @file DataBrokerConfiguration.cpp
 *
 */

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: Add logs

DDSRouterConfiguration::DDSRouterConfiguration(
        const RawConfiguration& raw_configuration)
    : raw_configuration_(raw_configuration)
{
    if (!raw_configuration_.IsMap() && !raw_configuration_.IsNull())
    {
        throw ConfigurationException("DDSRouter expects a map as base yaml type or an empty");
    }
}

DDSRouterConfiguration::~DDSRouterConfiguration()
{
}

std::list<std::shared_ptr<FilterTopic>> DDSRouterConfiguration::allowlist() const
{
    return generic_get_topic_list_(ALLOWLIST_TAG);
}

std::list<std::shared_ptr<FilterTopic>> DDSRouterConfiguration::blocklist() const
{
    return generic_get_topic_list_(BLOCKLIST_TAG);
}

std::map<ParticipantId, RawConfiguration> DDSRouterConfiguration::participants_configurations() const
{
    std::map<ParticipantId, RawConfiguration> result;

    try
    {
        for (YAML::const_iterator participant_it = raw_configuration_.begin();
                participant_it != raw_configuration_.end();
                ++participant_it)
        {
            std::string value_str = participant_it->first.as<std::string>();

            // Check if it is a valid name for a participant
            if (!ParticipantId::is_valid_id(value_str))
            {
                continue;
            }

            // Add new Participant with its configuration
            // NOTE: There will not be repeated Participant Ids as in a yaml the keys are unique
            result[value_str] = participant_it->second;
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning with e what
        throw ConfigurationException("Error while getting participant configurations in DDSRouter configuration.");
    }

    return result;
}

std::set<RealTopic> DDSRouterConfiguration::real_topics() const
{
    std::set<RealTopic> result;

    for (const std::shared_ptr<FilterTopic>& topic : generic_get_topic_list_(ALLOWLIST_TAG))
    {
        if (RealTopic::is_real_topic(topic->topic_name(), topic->topic_type()))
        {
            result.emplace(RealTopic(topic->topic_name(), topic->topic_type()));
        }
    }

    return result;
}

std::list<std::shared_ptr<FilterTopic>> DDSRouterConfiguration::generic_get_topic_list_(
        const char* list_tag) const
{
    // TODO: support regex topic

    std::list<std::shared_ptr<FilterTopic>> result;

    try
    {
        if (raw_configuration_[list_tag])
        {
            for (auto topic : raw_configuration_[list_tag])
            {
                std::string new_topic_name;
                std::string new_topic_type;

                if (topic[TOPIC_NAME_TAG])
                {
                    new_topic_name = topic[TOPIC_NAME_TAG].as<std::string>();
                }
                else
                {
                    // TODO: Add warning
                    // Not allowed topics without name
                    continue;
                }

                if (topic[TOPIC_TYPE_NAME_TAG])
                {
                    new_topic_type = topic[TOPIC_TYPE_NAME_TAG].as<std::string>();
                }

                if (new_topic_type.empty())
                {
                    result.push_back(
                        std::make_shared<WildcardTopic>(new_topic_name));
                }
                else
                {
                    result.push_back(
                        std::make_shared<WildcardTopic>(new_topic_name, new_topic_type));
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning
        throw ConfigurationException(
                  std::string("Error while getting topic list ") +
                  list_tag +
                  std::string(" in DDSRouter configuration."));
    }

    return result;
}

} /* namespace ddsrouter */
} /* namespace eprosima */

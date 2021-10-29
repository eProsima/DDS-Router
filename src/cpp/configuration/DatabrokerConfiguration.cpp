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

#include <databroker/configuration/DatabrokerConfiguration.hpp>
#include <databroker/types/configuration_tags.hpp>
#include <databroker/types/topic/WildcardTopic.hpp>
#include <databroker/exceptions/ConfigurationException.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs

DatabrokerConfiguration::DatabrokerConfiguration(
        const RawConfiguration& raw_configuration)
    : raw_configuration_(raw_configuration)
{
}

DatabrokerConfiguration::~DatabrokerConfiguration()
{
}

std::list<AbstractTopic*> DatabrokerConfiguration::whitelist() const
{
    return common_topic_list_get_(WHITELIST_TAG);
}

std::list<AbstractTopic*> DatabrokerConfiguration::blacklist() const
{
    return common_topic_list_get_(BLACKLIST_TAG);
}

std::map<ParticipantId, RawConfiguration> DatabrokerConfiguration::participants_configurations() const
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
            result[value_str] = participant_it->second;
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning with e what
        throw ConfigurationException("Error while getting participant configurations in Databroker configuration.");
    }

    return result;
}

std::list<AbstractTopic*> DatabrokerConfiguration::common_topic_list_get_(
        const char* list_tag) const
{
    // TODO: support regex topic

    std::list<AbstractTopic*> result;

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
                else
                {
                    new_topic_type = "*";
                }

                result.push_back(new WildcardTopic(new_topic_name, new_topic_type));
            }
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning
        throw ConfigurationException(
                  std::string("Error while getting topic list ") +
                  list_tag +
                  std::string(" in Databroker configuration."));
    }

    return result;
}

} /* namespace databroker */
} /* namespace eprosima */

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
 * @file YamlConfiguration_Topic.cpp
 *
 */

#include <ddsrouter/exception/ConfigurationException.hpp>
#include <ddsrouter/exception/UnsupportedException.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

std::shared_ptr<FilterTopic> YamlElementConfiguration::filter_topic(const Yaml& yaml)
{
    std::string name;
    std::string type;
    bool keyed;
    bool has_keyed_set = false;

    try
    {
        if (yaml[TOPIC_NAME_TAG])
        {
            name = yaml[TOPIC_NAME_TAG].as<std::string>();
        }
        else
        {
            // TODO: Add warning
            throw ConfigurationException(std::string(
                "Error while getting Filter Topic in DDSRouter configuration. Topic Name must be specified"));
        }

        if (yaml[TOPIC_TYPE_NAME_TAG])
        {
            type = yaml[TOPIC_TYPE_NAME_TAG].as<std::string>();
        }

        if (yaml[TOPIC_KIND_TAG])
        {
            has_keyed_set = true;
            keyed = yaml[TOPIC_KIND_TAG].as<bool>();
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning
        throw ConfigurationException(utils::Formatter() <<
                "Error while getting Filter Topic in DDSRouter configuration." << e.what());
    }

    if (type.empty())
    {
        return
            std::make_shared<WildcardTopic>(name, has_keyed_set, keyed);
    }
    else
    {
        return
            std::make_shared<WildcardTopic>(name, type, has_keyed_set, keyed);
    }
}

std::shared_ptr<RealTopic> YamlElementConfiguration::real_topic(const Yaml& yaml)
{
    std::string name;
    std::string type;
    bool keyed;
    bool has_keyed_set = false;

    try
    {
        if (yaml[TOPIC_NAME_TAG])
        {
            name = yaml[TOPIC_NAME_TAG].as<std::string>();
        }
        else
        {
            // TODO: Add warning
            throw ConfigurationException(std::string(
                "Error while getting Real Topic in DDSRouter configuration. Topic Name must be specified"));
        }

        if (yaml[TOPIC_TYPE_NAME_TAG])
        {
            type = yaml[TOPIC_TYPE_NAME_TAG].as<std::string>();
        }
        else
        {
            // TODO: Add warning
            throw ConfigurationException(std::string(
                "Error while getting Real Topic in DDSRouter configuration. Type name must be specified"));
        }

        if (yaml[TOPIC_KIND_TAG])
        {
            has_keyed_set = true;
            keyed = yaml[TOPIC_KIND_TAG].as<bool>();
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning
        throw ConfigurationException(utils::Formatter() <<
                "Error while getting Real Topic in DDSRouter configuration." << e.what());
    }

    if (has_keyed_set)
    {
        return std::make_shared<RealTopic>(name, type, keyed);
    }
    else
    {
        return std::make_shared<RealTopic>(name, type);
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

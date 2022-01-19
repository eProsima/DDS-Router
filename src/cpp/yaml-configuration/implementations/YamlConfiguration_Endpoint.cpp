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
 * @file DomainId_configuration.cpp
 *
 */

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

DomainId YamlElementConfiguration::domain_id(
    const Yaml& yaml,
    bool required /* = true */,
    bool default_value /* = false */)
{
    DomainIdType domain;

    if (yaml[DOMAIN_ID_TAG])
    {
        // Try to get domain value from yaml
        try
        {
            domain = yaml[DOMAIN_ID_TAG].as<DomainIdType>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Domain has incorrect format" << e.what());
        }
    }
    else
    {
        if (required)
        {
            throw ConfigurationException(
                    "Not domain tag found");
        }
        else
        {
            return DomainId(default_value);
        }
    }

    return DomainId(domain);
}

GuidPrefix YamlElementConfiguration::guid_prefix(const Yaml& yaml)
{
    uint32_t id;
    std::string guid;
    bool ros_id;
    bool id_set = false;
    bool ros_id_set = false;
    bool guid_set = false;

    // Check if tag for guid exists
    if (yaml[DISCOVERY_SERVER_GUID_TAG])
    {
        guid_set = true;

        // Try to get guid
        try
        {
            guid = yaml[DISCOVERY_SERVER_GUID_TAG].as<std::string>();
        }
        catch (const std::exception& e)
        {
            throw ConfigurationException(utils::Formatter() <<
                          "Guid Prefix has incorrect format" << e.what());
        }
    }
    else
    {
        // Check if tag for Ros exists
        if (yaml[DISCOVERY_SERVER_ID_ROS_TAG])
        {
            ros_id_set = true;

            // Try to get domain value from yaml
            try
            {
                ros_id = yaml[DISCOVERY_SERVER_ID_ROS_TAG].as<bool>();
            }
            catch (const std::exception& e)
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Ros Guid Prefix Tag has incorrect format. Must be boolean." << e.what());
            }
        }

        // Check if id exists, if not use default
        if (yaml[DISCOVERY_SERVER_ID_TAG])
        {
            id_set = true;

            // Try to get domain value from yaml
            try
            {
                id = yaml[DISCOVERY_SERVER_ID_TAG].as<uint32_t>();
            }
            catch (const std::exception& e)
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Guid Prefix Id has incorrect format. Must be uint32." << e.what());
            }
        }
    }

    if (guid_set)
    {
        return GuidPrefix(guid);
    }
    else
    {
        if (ros_id_set)
        {
            if (id_set)
            {
                return GuidPrefix(ros_id, id);
            }
            else
            {
                return GuidPrefix(ros_id);
            }
        }
        else
        {
            if (id_set)
            {
                return GuidPrefix(id);
            }
            else
            {
                return GuidPrefix();
            }
        }
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

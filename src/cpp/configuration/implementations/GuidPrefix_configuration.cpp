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
 * @file GuidPrefix_configuration.cpp
 *
 */

#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

GuidPrefix::GuidPrefix(
        const RawConfiguration& configuration)
{
    // Check if tag for guid exists
    if (configuration[DISCOVERY_SERVER_GUID_TAG])
    {
        // Try to get domain value from configuration
        try
        {
            *this =
                    GuidPrefix(configuration[DISCOVERY_SERVER_GUID_TAG].as<std::string>());
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
        bool ros_guid = false;
        if (configuration[DISCOVERY_SERVER_ID_ROS_TAG])
        {
            // Try to get domain value from configuration
            try
            {
                ros_guid = configuration[DISCOVERY_SERVER_ID_ROS_TAG].as<bool>();
            }
            catch (const std::exception& e)
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Ros Guid Prefix Tag has incorrect format. Must be boolean." << e.what());
            }
        }

        // Check if id exists, if not use default
        bool has_id = false;
        uint32_t id;
        if (configuration[DISCOVERY_SERVER_ID_TAG])
        {
            // Try to get domain value from configuration
            try
            {
                id = configuration[DISCOVERY_SERVER_ID_TAG].as<uint32_t>();
                has_id = true;
            }
            catch (const std::exception& e)
            {
                throw ConfigurationException(utils::Formatter() <<
                              "Guid Prefix Id has incorrect format. Must be uint32." << e.what());
            }
        }

        if (has_id)
        {
            *this = GuidPrefix(ros_guid, id);
        }
        else
        {
            *this = GuidPrefix(ros_guid);
        }
    }
}

RawConfiguration GuidPrefix::dump(
        RawConfiguration&) const
{
    // TODO
    throw UnsupportedException("GuidPrefix::dump is not supported yet.");
}

} /* namespace ddsrouter */
} /* namespace eprosima */

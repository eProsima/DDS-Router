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

#include <ddsrouter/exception/ConfigurationException.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

ParticipantId YamlParticipantConfiguration::participant_id(const Yaml& yaml)
{
    if (yaml[PARTICIPANT_NAME_TAG])
    {
        std::string name_str = yaml[PARTICIPANT_NAME_TAG].as<std::string>();
        return ParticipantId(name_str);
    }
    else
    {
        throw ConfigurationException("Id not specified.");
    }
}

ParticipantKind YamlParticipantConfiguration::participant_type(const Yaml& yaml)
{
    if (yaml[PARTICIPANT_TYPE_TAG])
    {
        std::string type_str = yaml[PARTICIPANT_TYPE_TAG].as<std::string>();
        return ParticipantKind::participant_type_from_name(type_str);
    }
    else
    {
        throw ConfigurationException("Type not specified.");
    }
}

std::shared_ptr<configuration::ParticipantConfiguration> YamlParticipantConfiguration::participant_configuration_factory(const Yaml& yaml)
{
    try
    {
        // Get Type. If this fails, configuration must fail
        ParticipantKind type = participant_type(yaml);

        switch (type())
        {
        case ParticipantKind::ECHO:
        case ParticipantKind::DUMMY:
            return std::make_shared<configuration::ParticipantConfiguration>(
                std_participant_configuration(yaml, type));

        case ParticipantKind::SIMPLE_RTPS:
            return std::make_shared<configuration::ParticipantConfiguration>(
                simple_participant_configuration(yaml, type));

        case ParticipantKind::LOCAL_DISCOVERY_SERVER:
        case ParticipantKind::WAN:
            return std::make_shared<configuration::ParticipantConfiguration>(
                discovery_server_participant_configuration(yaml, type));

        default:
            throw ConfigurationException(
                utils::Formatter() << "Unkown or non valid Participant type:" << type << ".");
            break;
        }
    }
    catch (const std::exception& e)
    {
        // TODO: Add Warning
        throw ConfigurationException(utils::Formatter() <<
                "Error while getting Participant configuration: " << e.what() << ".");
    }
}

configuration::ParticipantConfiguration YamlParticipantConfiguration::std_participant_configuration(
        const Yaml& yaml,
        ParticipantKind type)
{
    // If this fails, it should fail
    ParticipantId id = participant_id(yaml);

    return configuration::ParticipantConfiguration(id, type);
}

configuration::SimpleParticipantConfiguration YamlParticipantConfiguration::simple_participant_configuration(
        const Yaml& yaml,
        ParticipantKind type)
{
    // If this fails, it should fail
    ParticipantId id = participant_id(yaml);

    // If this fails, get default domain
    DomainId domain = YamlElementConfiguration::domain_id(yaml, false, false);
    return configuration::SimpleParticipantConfiguration(id, type, domain);
}

configuration::DiscoveryServerParticipantConfiguration YamlParticipantConfiguration::discovery_server_participant_configuration(
        const Yaml& yaml,
        ParticipantKind type)
{
    // Get Participant id/name. If this fails, it should fail
    ParticipantId id = participant_id(yaml);

    // Get domain. If this fails, get default domain
    DomainId domain = YamlElementConfiguration::domain_id(yaml, false, true);

    // Get DS guid. If this fails it should fail
    GuidPrefix guid;
    if (yaml[DISCOVERY_SERVER_GUID_PREFIX_TAG])
    {
        guid = YamlElementConfiguration::guid_prefix(yaml[DISCOVERY_SERVER_GUID_PREFIX_TAG]);
    }
    else
    {
        throw ConfigurationException("Discovery server guid prefix not specified.");
    }

    // Get listening addresses if present
    std::set<std::shared_ptr<Address>> listening_addresses;
    if (yaml[LISTENING_ADDRESSES_TAG])
    {
        for (auto yaml_address : yaml[LISTENING_ADDRESSES_TAG])
        {
            listening_addresses.insert(
                std::make_shared<Address>(YamlElementConfiguration::address(yaml_address)));
        }
    }

    // Get connection addresses if present
    std::set<std::shared_ptr<DiscoveryServerConnectionAddress>> connection_addresses;
    if (yaml[CONNECTION_ADDRESSES_TAG])
    {
        for (auto yaml_address : yaml[CONNECTION_ADDRESSES_TAG])
        {
            connection_addresses.insert(
                std::make_shared<DiscoveryServerConnectionAddress>(YamlElementConfiguration::discovery_server_connection_address(yaml_address)));
        }
    }

    // TLS configuration
    std::map<std::string, std::string> tls;
    // TODO

    return configuration::DiscoveryServerParticipantConfiguration(
        id, guid, listening_addresses, connection_addresses, tls, type, domain);
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#include <ddsrouter/security/tls/TlsConfiguration.hpp>
#include <ddsrouter/yaml-configuration/YamlConfiguration.hpp>
#include <ddsrouter/yaml-configuration/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

std::shared_ptr<configuration::ParticipantConfiguration> YamlParticipantConfiguration::participant_configuration_factory(const Yaml& yaml)
{
    try
    {
        // Get Type. If this fails, configuration must fail
        ParticipantKind type = YamlElementConfiguration::participant_type(yaml);

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
    ParticipantId id = YamlElementConfiguration::participant_id(yaml);

    return configuration::ParticipantConfiguration(id, type);
}

configuration::SimpleParticipantConfiguration YamlParticipantConfiguration::simple_participant_configuration(
        const Yaml& yaml,
        ParticipantKind type)
{
    // If this fails, it should fail
    ParticipantId id = YamlElementConfiguration::participant_id(yaml);

    // If this fails, get default domain
    DomainId domain = YamlElementConfiguration::domain_id(yaml, false, false);
    return configuration::SimpleParticipantConfiguration(id, type, domain);
}

configuration::DiscoveryServerParticipantConfiguration YamlParticipantConfiguration::discovery_server_participant_configuration(
        const Yaml& yaml,
        ParticipantKind type)
{
    // Non required arguments boolean to check if they have been set
    bool has_domain = false;
    bool has_tls = false;

    ParticipantId id;
    DomainId domain;
    GuidPrefix guid;
    std::set<std::shared_ptr<Address>> listening_addresses;
    std::set<std::shared_ptr<DiscoveryServerConnectionAddress>> connection_addresses;
    security::TlsConfiguration tls;

    // Get Participant id/name. If this fails, it should fail
    id = YamlElementConfiguration::participant_id(yaml);

    // Get domain. If this fails, get default domain
    if (yaml[DOMAIN_ID_TAG])
    {
        has_domain = true;
        domain = YamlElementConfiguration::domain_id(yaml);
    }

    // Get DS guid. If this fails it should fail
    if (yaml[DISCOVERY_SERVER_GUID_PREFIX_TAG])
    {
        guid = YamlElementConfiguration::guid_prefix(yaml[DISCOVERY_SERVER_GUID_PREFIX_TAG]);
    }
    else
    {
        throw ConfigurationException("Discovery server guid prefix not specified.");
    }

    // Get listening addresses if present
    if (yaml[LISTENING_ADDRESSES_TAG])
    {
        for (auto yaml_address : yaml[LISTENING_ADDRESSES_TAG])
        {
            listening_addresses.insert(
                std::make_shared<Address>(YamlElementConfiguration::address(yaml_address)));
        }
    }

    // Get connection addresses if present
    if (yaml[CONNECTION_ADDRESSES_TAG])
    {
        for (auto yaml_address : yaml[CONNECTION_ADDRESSES_TAG])
        {
            connection_addresses.insert(
                std::make_shared<DiscoveryServerConnectionAddress>(YamlElementConfiguration::discovery_server_connection_address(yaml_address)));
        }
    }

    // TLS configuration
    if (yaml[TLS_TAG])
    {
        has_tls = true;
        tls = YamlElementConfiguration::tls_configuration(yaml[TLS_TAG]);
    }

    if (has_tls)
    {
        if (has_domain)
        {
            return configuration::DiscoveryServerParticipantConfiguration(
                id, guid, listening_addresses, connection_addresses, type, tls, domain);
        }
        else
        {
            return configuration::DiscoveryServerParticipantConfiguration(
                id, guid, listening_addresses, connection_addresses, type, tls);
        }
    }
    else
    {
        return configuration::DiscoveryServerParticipantConfiguration(
            id, guid, listening_addresses, connection_addresses, type);
    }
    // TODO: cannot use domain and not TLS (ups)
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

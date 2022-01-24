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
 * @file YamlReaderParticipant.cpp
 *
 */

#include <ddsrouter/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/participant/SimpleParticipantConfiguration.hpp>

#include <ddsrouter/security/tls/TlsConfiguration.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/yaml/YamlReader.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

template <>
configuration::ParticipantConfiguration YamlReader::get<configuration::ParticipantConfiguration>(const Yaml& yml)
{
    // Id required
    ParticipantId id = get<ParticipantId>(yml, PARTICIPANT_NAME_TAG);

    // Kind required
    ParticipantKind kind = get<ParticipantKind>(yml, PARTICIPANT_KIND_TAG);

    return configuration::ParticipantConfiguration(id, kind);
}

template <>
configuration::SimpleParticipantConfiguration YamlReader::get<configuration::SimpleParticipantConfiguration>(const Yaml& yml)
{
    // Id required
    ParticipantId id = get<ParticipantId>(yml, PARTICIPANT_NAME_TAG);

    // Kind required
    ParticipantKind kind = get<ParticipantKind>(yml, PARTICIPANT_KIND_TAG);

    // Domain required
    DomainId domain = get<DomainId>(yml, DOMAIN_ID_TAG);

    return configuration::SimpleParticipantConfiguration(id, kind, domain);
}

template <>
configuration::DiscoveryServerParticipantConfiguration YamlReader::get<configuration::DiscoveryServerParticipantConfiguration>(const Yaml& yml)
{
    // Id required
    ParticipantId id = get<ParticipantId>(yml, PARTICIPANT_NAME_TAG);

    // Kind required
    ParticipantKind kind = get<ParticipantKind>(yml, PARTICIPANT_KIND_TAG);

    // Guid Prefix required
    GuidPrefix guid = get<GuidPrefix>(yml, DISCOVERY_SERVER_GUID_PREFIX_TAG);

    // Domain option
    DomainId domain;
    bool has_domain = is_tag_present(yml, DOMAIN_ID_TAG);
    if (has_domain)
    {
        domain = get<DomainId>(yml, DOMAIN_ID_TAG);
    }

    // Optional listening addresses
    std::set<Address> listening_addresses;
    if (is_tag_present(yml, LISTENING_ADDRESSES_TAG))
    {
        listening_addresses = get_set<Address>(yml, LISTENING_ADDRESSES_TAG);
    }

    // Optional connection addresses
    std::set<DiscoveryServerConnectionAddress> connection_addresses;
    if (is_tag_present(yml, CONNECTION_ADDRESSES_TAG))
    {
        connection_addresses = get_set<DiscoveryServerConnectionAddress>(yml, CONNECTION_ADDRESSES_TAG);
    }

    // Optional TLS
    std::shared_ptr<security::TlsConfiguration> tls;
    bool has_tls = is_tag_present(yml, TLS_TAG);
    if (has_tls)
    {
        tls = get<std::shared_ptr<security::TlsConfiguration>>(yml, TLS_TAG);
    }

    if (has_domain)
    {
        if (has_tls)
        {
            return configuration::DiscoveryServerParticipantConfiguration(
                id,
                guid,
                listening_addresses,
                connection_addresses,
                kind,
                tls,
                domain);
        }
        else
        {
            return configuration::DiscoveryServerParticipantConfiguration(
                id,
                guid,
                listening_addresses,
                connection_addresses,
                domain,
                kind);
        }
    }
    else
    {
        if (has_tls)
        {
            return configuration::DiscoveryServerParticipantConfiguration(
                id,
                guid,
                listening_addresses,
                connection_addresses,
                kind,
                tls);
        }
        else
        {
            return configuration::DiscoveryServerParticipantConfiguration(
                id,
                guid,
                listening_addresses,
                connection_addresses,
                kind);
        }
    }
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

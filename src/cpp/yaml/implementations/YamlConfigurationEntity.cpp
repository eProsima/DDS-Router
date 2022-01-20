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
 * @file Address_configuration.cpp
 *
 */

#include <ddsrouter/security/tls/TlsConfiguration.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter/types/endpoint/DomainId.hpp>
#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/topic/WildcardTopic.hpp>
#include <ddsrouter/yaml/YamlConfiguration.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

template <>
TransportProtocol YamlConfiguration::get<TransportProtocol>(const Yaml& yml)
{
    return get_enumeration<TransportProtocol>(
        yml,
        ADDRESS_TRANSPORT_TAG,
        {
            {ADDRESS_TRANSPORT_TCP_TAG, TransportProtocol::TCP},
            {ADDRESS_TRANSPORT_UDP_TAG, TransportProtocol::UDP},
        });
}

template <>
IpVersion YamlConfiguration::get<IpVersion>(const Yaml& yml)
{
    return get_enumeration<IpVersion>(
        yml,
        ADDRESS_IP_VERSION_TAG,
        {
            {ADDRESS_IP_VERSION_V4_TAG, IpVersion::IPv4},
            {ADDRESS_IP_VERSION_V6_TAG, IpVersion::IPv6},
        });
}

template <>
ParticipantId YamlConfiguration::get<ParticipantId>(const Yaml& yml)
{
    // Participant name required
    return ParticipantId(get_native<std::string>(yml, PARTICIPANT_NAME_TAG));
}

template <>
ParticipantKind YamlConfiguration::get<ParticipantKind>(const Yaml& yml)
{
    // Participant kind required
    std::string kind = get_native<std::string>(yml,  PARTICIPANT_KIND_TAG);
    return ParticipantKind:: participant_kind_from_name(kind);
}

template <>
DomainId YamlConfiguration::get<DomainId>(const Yaml& yml)
{
    // Domain id required
    return DomainId(get_native<DomainIdType>(yml, DOMAIN_ID_TAG));
}

template <>
GuidPrefix YamlConfiguration::get<GuidPrefix>(const Yaml& yml)
{
    // If guid exists, use it. Non mandatory.
    if (is_tag_present(yml, DISCOVERY_SERVER_GUID_TAG))
    {
        std::string guid = get_native<std::string>(yml, DISCOVERY_SERVER_GUID_TAG);
        return GuidPrefix(guid);
    }

    // ROS DS is optional.
    bool ros_id;
    bool ros_id_set = is_tag_present(yml, DISCOVERY_SERVER_ID_ROS_TAG);
    if (ros_id_set)
    {
        ros_id = get_native<bool>(yml, DISCOVERY_SERVER_ID_ROS_TAG);
    }

    // Id is mandatory if guid is not present
    uint32_t id = get_native<bool>(yml, DISCOVERY_SERVER_ID_TAG);

    // Create GuidPrefix
    if (ros_id_set)
    {
        return GuidPrefix(ros_id, id);
    }
    else
    {
        return GuidPrefix(id);
    }
}

template <>
Address YamlConfiguration::get<Address>(const Yaml& yml)
{
    // TODO
    return Address();
}

template <>
DiscoveryServerConnectionAddress YamlConfiguration::get<DiscoveryServerConnectionAddress>(const Yaml& yml)
{
    // GuidPrefix required
    GuidPrefix server_guid = get<GuidPrefix>(yml, DISCOVERY_SERVER_GUID_TAG);

    // Addresses required
    std::set<Address> addresses = get_set<Address>(yml, COLLECTION_ADDRESSES_TAG);

    // Create Connection Address
    return DiscoveryServerConnectionAddress(server_guid, addresses);
}

template <>
RealTopic YamlConfiguration::get<RealTopic>(const Yaml& yml)
{
    // Mandatory name
    std::string name = get_native<std::string>(yml, TOPIC_NAME_TAG);

    // Mandatory type
    std::string type = get_native<std::string>(yml, TOPIC_TYPE_NAME_TAG);

    // Optional keyed
    bool keyed;
    bool keyed_set = is_tag_present(yml, TOPIC_KIND_TAG);
    if (keyed_set)
    {
        keyed = get_native<bool>(yml, TOPIC_KIND_TAG);
    }

    // Create Topic
    if (keyed_set)
    {
        return RealTopic(name, type, keyed);
    }
    else
    {
        return RealTopic(name, type);
    }
}

template <>
WildcardTopic YamlConfiguration::get<WildcardTopic>(const Yaml& yml)
{
    // Mandatory name
    std::string name = get_native<std::string>(yml, TOPIC_NAME_TAG);

    // Optional type
    std::string type;
    bool type_set = is_tag_present(yml, TOPIC_TYPE_NAME_TAG);
    if (type_set)
    {
        type = get_native<std::string>(yml, TOPIC_TYPE_NAME_TAG);
    }

    // Optional keyed
    bool keyed;
    bool keyed_set = is_tag_present(yml, TOPIC_KIND_TAG);
    if (keyed_set)
    {
        keyed = get_native<bool>(yml, TOPIC_KIND_TAG);
    }

    // Create Topic
    if (keyed_set)
    {
        if (type_set)
        {
            return WildcardTopic(name, type, true, keyed);
        }
        else
        {
            return WildcardTopic(name, true, keyed);
        }
    }
    else
    {
        if (type_set)
        {
            return WildcardTopic(name, type, false);
        }
        else
        {
            return WildcardTopic(name, false);
        }
    }
}

template <>
security::TlsConfiguration YamlConfiguration::get<security::TlsConfiguration>(const Yaml& yml)
{
    // TODO
    return security::TlsConfiguration();
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

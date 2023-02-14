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

#pragma once

#include <sstream>

#include <ddspipe_core/types/dds/DomainId.hpp>
#include <ddspipe_core/types/dds/GuidPrefix.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/testing/random_values.hpp>

#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_participants/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddspipe_participants/testing/random_values.hpp>

#include <ddspipe_yaml/Yaml.hpp>
#include <ddspipe_yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddspipe {
namespace yaml {
namespace testing {

template <typename T>
struct YamlField
{
    YamlField()
        : present(false)
    {
    }

    YamlField(
            const T& arg_value,
            bool arg_present = true)
        : value(arg_value)
        , present(arg_present)
    {
    }

    T value;
    bool present;
};

template <typename T>
void add_field_to_yaml(
        Yaml& yml,
        const YamlField<T>& field,
        const std::string& tag)
{
    if (field.present)
    {
        yml[tag] = field.value;
    }
}

void guid_prefix_to_yaml(
        Yaml& yml,
        const core::types::GuidPrefix& guid_prefix)
{
    std::stringstream ss;
    ss << guid_prefix;

    add_field_to_yaml(
        yml,
        YamlField<std::string>(ss.str()),
        DISCOVERY_SERVER_GUID_TAG);
}

void discovery_server_guid_prefix_to_yaml(
        Yaml& yml,
        const core::types::GuidPrefix& guid_prefix)
{
    Yaml yml_guid;
    guid_prefix_to_yaml(yml_guid, guid_prefix);

    yml[DISCOVERY_SERVER_GUID_PREFIX_TAG] = yml_guid;
}

void address_to_yaml(
        Yaml& yml,
        const participants::types::Address& address)
{
    add_field_to_yaml(
        yml,
        YamlField<participants::types::IpType>(address.ip()),
        ADDRESS_IP_TAG);

    add_field_to_yaml(
        yml,
        YamlField<participants::types::PortType>(address.port()),
        ADDRESS_PORT_TAG);

    if (address.transport_protocol() == participants::types::TransportProtocol::udp)
    {
        add_field_to_yaml(
            yml,
            YamlField<std::string>(ADDRESS_TRANSPORT_UDP_TAG),
            ADDRESS_TRANSPORT_TAG);
    }
    else if (address.transport_protocol() == participants::types::TransportProtocol::tcp)
    {
        add_field_to_yaml(
            yml,
            YamlField<std::string>(ADDRESS_TRANSPORT_TCP_TAG),
            ADDRESS_TRANSPORT_TAG);
    }

    if (address.ip_version() == participants::types::IpVersion::v4)
    {
        add_field_to_yaml(
            yml,
            YamlField<std::string>(ADDRESS_IP_VERSION_V4_TAG),
            ADDRESS_IP_VERSION_TAG);
    }
    else if (address.ip_version() == participants::types::IpVersion::v6)
    {
        add_field_to_yaml(
            yml,
            YamlField<std::string>(ADDRESS_IP_VERSION_V6_TAG),
            ADDRESS_IP_VERSION_TAG);
    }
}

void participantid_to_yaml(
        Yaml& yml,
        const core::types::ParticipantId& id)
{
    add_field_to_yaml(
        yml,
        YamlField<std::string>(id),
        PARTICIPANT_NAME_TAG);
}

void domain_to_yaml(
        Yaml& yml,
        const core::types::DomainId& domain)
{
    add_field_to_yaml(
        yml,
        YamlField<core::types::DomainIdType>(domain.domain_id),
        DOMAIN_ID_TAG);
}

void repeater_to_yaml(
        Yaml& yml,
        const bool& repeater)
{
    add_field_to_yaml(
        yml,
        YamlField<bool>(repeater),
        IS_REPEATER_TAG);
}

// // Create a yaml QoS object only with reliability
// void qos_to_yaml(
//         Yaml& yml,
//         const TopicQoS& qos)
// {
//     // TODO: extend this for all qos
//     add_field_to_yaml(yml, YamlField<bool>(qos.is_reliable()), QOS_RELIABLE_TAG);
// }

// // Create a yaml Topic object with name, type and key tags
// void filter_topic_to_yaml(
//         Yaml& yml,
//         const WildcardDdsFilterTopic& topic)
// {
//     if (topic.topic_name.is_set())
//     {
//         add_field_to_yaml(yml, YamlField<std::string>(topic.topic_name), TOPIC_NAME_TAG);
//     }

//     if (topic.type_name.is_set())
//     {
//         add_field_to_yaml(yml, YamlField<std::string>(topic.type_name), TOPIC_TYPE_NAME_TAG);
//     }
// }

// // Create a yaml DdsTopic object with name, type, key and reliable tags
// void real_topic_to_yaml(
//         Yaml& yml,
//         const DdsTopic& topic,
//         const YamlField<std::string>& type,
//         const YamlField<Yaml>& qos)
// {
//     add_field_to_yaml(yml, YamlField<std::string>(topic.m_topic_name), TOPIC_NAME_TAG);
//     add_field_to_yaml(yml, YamlField<std::string>(topic.type_name), TOPIC_TYPE_NAME_TAG);
//     add_field_to_yaml(yml, YamlField<Yaml>(topic.topic_qos), TOPIC_QOS_TAG);
// }

} /* namespace testing */
} /* namespace yaml */
} /* namespace ddspipe */
} /* namespace eprosima */

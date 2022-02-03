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
 * @file YamlConfigurationTestUtils.hpp
 */

#ifndef _DDSROUTER_TEST_UNITTEST_YAML_YAMLCONFIGURATIONTESTUTILS_HPP_
#define _DDSROUTER_TEST_UNITTEST_YAML_YAMLCONFIGURATIONTESTUTILS_HPP_

#include <sstream>

#include <ddsrouter/types/endpoint/GuidPrefix.hpp>
#include <ddsrouter/types/address/Address.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/yaml/Yaml.hpp>
#include <ddsrouter/yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {
namespace test {

template <typename T>
struct YamlField
{
    YamlField()
        : present(false)
    {
    }

    YamlField(
            T arg_value)
        : value(arg_value)
        , present(true)
    {
    }

    bool present;
    T value;
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
        const GuidPrefix& guid_prefix)
{
    std::stringstream ss;
    ss << guid_prefix;

    test::add_field_to_yaml(
        yml,
        test::YamlField<std::string>(ss.str()),
        DISCOVERY_SERVER_GUID_TAG);
}

void address_to_yaml(
        Yaml& yml,
        const Address& address)
{
    test::add_field_to_yaml(
        yml,
        test::YamlField<IpType>(address.ip()),
        ADDRESS_IP_TAG);

    test::add_field_to_yaml(
        yml,
        test::YamlField<PortType>(address.port()),
        ADDRESS_PORT_TAG);

    if (address.transport_protocol() == UDP)
    {
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_TRANSPORT_UDP_TAG),
            ADDRESS_TRANSPORT_TAG);
    }
    else if (address.transport_protocol() == TCP)
    {
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_TRANSPORT_TCP_TAG),
            ADDRESS_TRANSPORT_TAG);
    }

    if (address.ip_version() == IPv4)
    {
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_IP_VERSION_V4_TAG),
            ADDRESS_IP_VERSION_TAG);
    }
    else if (address.ip_version() == IPv6)
    {
        test::add_field_to_yaml(
            yml,
            test::YamlField<std::string>(ADDRESS_IP_VERSION_V6_TAG),
            ADDRESS_IP_VERSION_TAG);
    }
}

void participantid_to_yaml(
        Yaml& yml,
        const ParticipantId& id)
{
    test::add_field_to_yaml(
        yml,
        test::YamlField<std::string>(id.id_name()),
        PARTICIPANT_NAME_TAG);
}

void participantkind_to_yaml(
        Yaml& yml,
        const ParticipantKind& kind)
{
    test::add_field_to_yaml(
        yml,
        test::YamlField<std::string>(kind.to_string()),
        PARTICIPANT_KIND_TAG);
}

} /* namespace test */
} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TEST_UNITTEST_YAML_YAMLCONFIGURATIONTESTUTILS_HPP_ */

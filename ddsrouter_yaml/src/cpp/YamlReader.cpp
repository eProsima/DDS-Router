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
 * @file YamlReader.cpp
 *
 */

#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/InitialPeersParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/EchoParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/types/address/Address.hpp>
#include <ddsrouter_core/types/address/DiscoveryServerConnectionAddress.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_core/types/dds/GuidPrefix.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>
#include <ddsrouter_core/types/security/tls/TlsConfiguration.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>
#include <ddsrouter_core/types/topic/WildcardTopic.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_yaml/Yaml.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>
#include <ddsrouter_yaml/yaml_configuration_tags.hpp>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/************************
* GENERIC              *
************************/

bool YamlReader::is_tag_present(
        const Yaml& yml,
        const TagType& tag)
{
    if (!yml.IsMap() && !yml.IsNull())
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Trying to find a tag: <" << tag << "> in a not yaml object map.");
    }

    // Explicit conversion to avoid windows format warning
    // This method performace is the same as only retrieving bool
    return (yml[tag]) ? true : false;
}

Yaml YamlReader::get_value_in_tag(
        const Yaml& yml,
        const TagType& tag)
{
    if (is_tag_present(yml, tag))
    {
        return yml[tag];
    }
    else
    {
        throw utils::ConfigurationException(
                  utils::Formatter() << "Required tag not found: <" << tag << ">.");
    }
}

template <>
YamlReaderVersion YamlReader::get<YamlReaderVersion>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    return get_enumeration<YamlReaderVersion>(
        yml,
                {
                    {VERSION_TAG_V_1_0, YamlReaderVersion::V_1_0},
                    {VERSION_TAG_V_2_0, YamlReaderVersion::V_2_0},
                    {VERSION_TAG_V_3_0, YamlReaderVersion::V_3_0},
                });
}

/************************
* STANDARD             *
************************/

template <>
int YamlReader::get<int>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    return get_scalar<int>(yml);
}

template <>
unsigned int YamlReader::get<unsigned int>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    return get_scalar<unsigned int>(yml);
}

template <>
bool YamlReader::get<bool>(
        const Yaml& yml,
        const YamlReaderVersion version /* version */)
{
    bool value = get_scalar<bool>(yml);
    return get_scalar<bool>(yml);
}

/************************
* ENTITIES             *
************************/

template <>
TransportProtocol YamlReader::get<TransportProtocol>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    return get_enumeration<TransportProtocol>(
        yml,
                {
                    {ADDRESS_TRANSPORT_TCP_TAG, TransportProtocol::tcp},
                    {ADDRESS_TRANSPORT_UDP_TAG, TransportProtocol::udp},
                });
}

template <>
IpVersion YamlReader::get<IpVersion>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    return get_enumeration<IpVersion>(
        yml,
                {
                    {ADDRESS_IP_VERSION_V4_TAG, IpVersion::v4},
                    {ADDRESS_IP_VERSION_V6_TAG, IpVersion::v6},
                });
}

template <>
PortType YamlReader::get<PortType>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Domain id required
    return PortType(get_scalar<PortType>(yml));
}

template <>
IpType YamlReader::get<IpType>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Domain id required
    return IpType(get_scalar<IpType>(yml));
}

template <>
ParticipantId YamlReader::get<ParticipantId>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Participant name required
    return ParticipantId(get_scalar<std::string>(yml));
}

template <>
ParticipantKind YamlReader::get<ParticipantKind>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Participant kind required
    ParticipantKind kind = participant_kind_from_name(get_scalar<std::string>(yml));

    // In version lower than 3.0 wan means wan_discovery_server
    if (version <= V_2_0 && kind == ParticipantKind::wan_initial_peers)
    {
        kind = ParticipantKind::wan_discovery_server;
    }

    return kind;
}

template <>
DomainId YamlReader::get<DomainId>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Domain id required
    return DomainId(get_scalar<DomainIdType>(yml));
}

template <>
GuidPrefix YamlReader::get<GuidPrefix>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // If guid exists, use it. Non mandatory.
    if (is_tag_present(yml, DISCOVERY_SERVER_GUID_TAG))
    {
        std::string guid = get_scalar<std::string>(yml, DISCOVERY_SERVER_GUID_TAG);
        return GuidPrefix(guid);
    }

    // ROS DS is optional.
    bool ros_id;
    bool ros_id_set = is_tag_present(yml, DISCOVERY_SERVER_ID_ROS_TAG);
    if (ros_id_set)
    {
        ros_id = get_scalar<bool>(yml, DISCOVERY_SERVER_ID_ROS_TAG);
    }

    // Id is mandatory if guid is not present
    uint32_t id = get_scalar<uint32_t>(yml, DISCOVERY_SERVER_ID_TAG);

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
Address YamlReader::get<Address>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Optional get IP version
    IpVersion ip_version;
    bool ip_version_set = is_tag_present(yml, ADDRESS_IP_VERSION_TAG);
    if (ip_version_set)
    {
        // Get IP Version from enumeration
        ip_version = get<IpVersion>(yml, ADDRESS_IP_VERSION_TAG, version);
    }

    // Optional get IP
    IpType ip;
    bool ip_set = is_tag_present(yml, ADDRESS_IP_TAG);
    if (ip_set)
    {
        ip = get<IpType>(yml, ADDRESS_IP_TAG, version);
    }

    // Optional get Domain tag for DNS
    std::string domain_name;
    bool domain_name_set = is_tag_present(yml, ADDRESS_DNS_TAG);
    if (domain_name_set)
    {
        domain_name = get_scalar<std::string>(yml, ADDRESS_DNS_TAG);
    }

    // If IP and domain_name set, warning that domain_name will not be used
    // If only domain_name set, get DNS response
    // If neither set, get default
    if (ip_set && domain_name_set)
    {
        logWarning(DDSROUTER_YAML,
                "Tag <" << ADDRESS_DNS_TAG << "> will not be used as <" << ADDRESS_IP_TAG << "> is set.");
        domain_name_set = false;
    }
    else if (!ip_set && !domain_name_set)
    {
        throw utils::ConfigurationException(utils::Formatter() <<
                      "Address requires to specify <" << ADDRESS_IP_TAG << "> or <" << ADDRESS_DNS_TAG << ">.");
    }

    // Optional get port
    PortType port;
    bool port_set = is_tag_present(yml, ADDRESS_PORT_TAG);
    if (port_set)
    {
        port = get<PortType>(yml, ADDRESS_PORT_TAG, version);
    }
    else
    {
        port = Address::default_port();
    }

    // Optional get Transport protocol
    TransportProtocol tp;
    bool tp_set = is_tag_present(yml, ADDRESS_TRANSPORT_TAG);
    if (tp_set)
    {
        tp = get<TransportProtocol>(yml, ADDRESS_TRANSPORT_TAG, version);
    }
    else
    {
        tp = Address::default_transport_protocol();
    }

    // Construct Address object
    if (domain_name_set)
    {
        if (ip_version_set)
        {
            return Address(port, ip_version, domain_name, tp);
        }
        else
        {
            return Address(port, domain_name, tp);
        }
    }
    else
    {
        if (ip_version_set)
        {
            return Address(ip, port, ip_version, tp);
        }
        else
        {
            return Address(ip, port, tp);
        }
    }
}

DiscoveryServerConnectionAddress _get_discovery_server_connection_address_v1(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // GuidPrefix required
    GuidPrefix server_guid = YamlReader::get<GuidPrefix>(yml, version);

    // Addresses required
    std::set<Address> addresses = YamlReader::get_set<Address>(yml, COLLECTION_ADDRESSES_TAG, version);

    // Create Connection Address
    return DiscoveryServerConnectionAddress(server_guid, addresses);
}

DiscoveryServerConnectionAddress _get_discovery_server_connection_address_latest(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // GuidPrefix required
    GuidPrefix server_guid = YamlReader::get<GuidPrefix>(yml, DISCOVERY_SERVER_GUID_PREFIX_TAG, version);

    // Addresses required
    std::set<Address> addresses = YamlReader::get_set<Address>(yml, COLLECTION_ADDRESSES_TAG, version);

    // Create Connection Address
    return DiscoveryServerConnectionAddress(server_guid, addresses);
}

template <>
DiscoveryServerConnectionAddress YamlReader::get<DiscoveryServerConnectionAddress>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    switch (version)
    {
        case V_1_0:
            return _get_discovery_server_connection_address_v1(yml, version);

        default:
            return _get_discovery_server_connection_address_latest(yml, version);
    }
}

template <>
RealTopic YamlReader::get<RealTopic>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Mandatory name
    std::string name = get_scalar<std::string>(yml, TOPIC_NAME_TAG);

    // Mandatory type
    std::string type = get_scalar<std::string>(yml, TOPIC_TYPE_NAME_TAG);

    // Optional keyed
    bool keyed;
    bool keyed_set = is_tag_present(yml, TOPIC_KIND_TAG);
    if (keyed_set)
    {
        keyed = get_scalar<bool>(yml, TOPIC_KIND_TAG);
    }

    // Optional reliable DataReader
    bool reliable;
    bool reliable_set = is_tag_present(yml, TOPIC_RELIABLE_TAG);
    if (reliable_set)
    {
        reliable = get_scalar<bool>(yml, TOPIC_RELIABLE_TAG);
    }

    if (keyed_set)
    {
        if (reliable_set)
        {
            return RealTopic(name, type, keyed, reliable);
        }
        else
        {
            return RealTopic(name, type, keyed);
        }
    }
    else
    {
        if (reliable_set)
        {
            return RealTopic(reliable, name, type);
        }
        else
        {
            return RealTopic(name, type);
        }
    }
}

template <>
WildcardTopic YamlReader::get<WildcardTopic>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Mandatory name
    std::string name = get_scalar<std::string>(yml, TOPIC_NAME_TAG);

    // Optional type
    std::string type;
    bool type_set = is_tag_present(yml, TOPIC_TYPE_NAME_TAG);
    if (type_set)
    {
        type = get_scalar<std::string>(yml, TOPIC_TYPE_NAME_TAG);
    }

    // Optional keyed
    bool keyed;
    bool keyed_set = is_tag_present(yml, TOPIC_KIND_TAG);
    if (keyed_set)
    {
        keyed = get_scalar<bool>(yml, TOPIC_KIND_TAG);
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
security::TlsConfiguration YamlReader::get<security::TlsConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion /* version */)
{
    // Optional private key
    std::string private_key_file;
    bool has_private_key_file = is_tag_present(yml, TLS_PRIVATE_KEY_TAG);
    if (has_private_key_file)
    {
        private_key_file = get_scalar<std::string>(yml, TLS_PRIVATE_KEY_TAG);
    }

    // Optional private key password
    std::string private_key_file_password;
    bool has_private_key_file_password = is_tag_present(yml, TLS_PASSWORD_TAG);
    if (has_private_key_file_password)
    {
        private_key_file_password = get_scalar<std::string>(yml, TLS_PASSWORD_TAG);
    }

    // Optional certificate authority
    std::string certificate_authority_file;
    bool has_certificate_authority_file = is_tag_present(yml, TLS_CA_TAG);
    if (has_certificate_authority_file)
    {
        certificate_authority_file = get_scalar<std::string>(yml, TLS_CA_TAG);
    }

    // Optional certificate chain
    std::string certificate_chain_file;
    bool has_certificate_chain_file = is_tag_present(yml, TLS_CERT_TAG);
    if (has_certificate_chain_file)
    {
        certificate_chain_file = get_scalar<std::string>(yml, TLS_CERT_TAG);
    }

    // Optional dh params
    std::string dh_params_file;
    bool has_dh_params_file = is_tag_present(yml, TLS_DHPARAMS_TAG);
    if (has_dh_params_file)
    {
        dh_params_file = get_scalar<std::string>(yml, TLS_DHPARAMS_TAG);
    }

    if (has_private_key_file && has_certificate_chain_file && has_dh_params_file)
    {
        if (has_certificate_authority_file)
        {
            // Both TLS configuration
            return security::TlsConfiguration(
                certificate_authority_file,
                private_key_file_password,
                private_key_file,
                certificate_chain_file,
                dh_params_file);
        }
        else
        {
            // Server TLS configuration
            return security::TlsConfiguration(
                private_key_file_password,
                private_key_file,
                certificate_chain_file,
                dh_params_file);
        }
    }
    else
    {
        if (has_certificate_authority_file)
        {
            // Client TLS configuration
            return security::TlsConfiguration(certificate_authority_file);
        }
        else
        {
            throw utils::ConfigurationException(
                      "TLS Configuration is set and does not fit with Client or Server parameters.");
        }
    }
}

/************************
* PARTICIPANTS         *
************************/

//////////////////////////////////
// ParticipantConfiguration
template <>
void YamlReader::fill(
        configuration::ParticipantConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Id required
    object.id = get<types::ParticipantId>(yml, PARTICIPANT_NAME_TAG, version);

    // Kind required
    object.kind = get<types::ParticipantKind>(yml, PARTICIPANT_KIND_TAG, version);
}

template <>
configuration::ParticipantConfiguration YamlReader::get(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    configuration::ParticipantConfiguration object;
    fill<configuration::ParticipantConfiguration>(object, yml, version);
    return object;
}

//////////////////////////////////
// EchoParticipantConfiguration
template <>
void YamlReader::fill(
        configuration::EchoParticipantConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Parent class fill
    fill<configuration::ParticipantConfiguration>(object, yml, version);

    // data optional
    if (is_tag_present(yml, ECHO_DATA_TAG))
    {
        object.echo_data = get<bool>(yml, ECHO_DATA_TAG, version);
    }

    // discovery optional
    if (is_tag_present(yml, ECHO_DISCOVERY_TAG))
    {
        object.echo_discovery = get<bool>(yml, ECHO_DISCOVERY_TAG, version);
    }

    // verbose optional
    if (is_tag_present(yml, ECHO_VERBOSE_TAG))
    {
        object.verbose = get<bool>(yml, ECHO_VERBOSE_TAG, version);
    }
}

template <>
configuration::EchoParticipantConfiguration YamlReader::get(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    configuration::EchoParticipantConfiguration object;
    fill<configuration::EchoParticipantConfiguration>(object, yml, version);
    return object;
}

//////////////////////////////////
// SimpleParticipantConfiguration
template <>
void YamlReader::fill(
        configuration::SimpleParticipantConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Parent class fill
    fill<configuration::ParticipantConfiguration>(object, yml, version);

    // Domain optional
    if (is_tag_present(yml, DOMAIN_ID_TAG))
    {
        object.domain = get<types::DomainId>(yml, DOMAIN_ID_TAG, version);
    }
}

template <>
configuration::SimpleParticipantConfiguration YamlReader::get(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    configuration::SimpleParticipantConfiguration object;
    fill<configuration::SimpleParticipantConfiguration>(object, yml, version);
    return object;
}

//////////////////////////////////
// DiscoveryServerParticipantConfiguration
template <>
void YamlReader::fill(
        configuration::DiscoveryServerParticipantConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Parent class fill
    fill<configuration::SimpleParticipantConfiguration>(object, yml, version);

    // Optional listening addresses
    if (YamlReader::is_tag_present(yml, LISTENING_ADDRESSES_TAG))
    {
        object.listening_addresses = YamlReader::get_set<types::Address>(yml, LISTENING_ADDRESSES_TAG, version);
    }

    // Optional connection addresses
    if (YamlReader::is_tag_present(yml, CONNECTION_ADDRESSES_TAG))
    {
        object.connection_addresses = YamlReader::get_set<types::DiscoveryServerConnectionAddress>(
            yml,
            CONNECTION_ADDRESSES_TAG,
            version);
    }

    // Optional TLS
    if (YamlReader::is_tag_present(yml, TLS_TAG))
    {
        object.tls_configuration = YamlReader::get<types::security::TlsConfiguration>(yml, TLS_TAG, version);
    }

    // NOTE: The only field that change regarding the version is the GuidPrefix.
    switch (version)
    {
        case V_1_0:
            object.discovery_server_guid_prefix =
                    YamlReader::get<types::GuidPrefix>(yml, version);
            break;

        default:
            object.discovery_server_guid_prefix =
                    YamlReader::get<types::GuidPrefix>(yml, DISCOVERY_SERVER_GUID_PREFIX_TAG, version);
            break;
    }
}

template <>
configuration::DiscoveryServerParticipantConfiguration YamlReader::get(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    configuration::DiscoveryServerParticipantConfiguration object;
    fill<configuration::DiscoveryServerParticipantConfiguration>(object, yml, version);
    return object;
}

//////////////////////////////////
// InitialPeersParticipantConfiguration
template <>
void YamlReader::fill(
        configuration::InitialPeersParticipantConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Parent class fill
    fill<configuration::SimpleParticipantConfiguration>(object, yml, version);

    // Optional listening addresses
    if (YamlReader::is_tag_present(yml, LISTENING_ADDRESSES_TAG))
    {
        object.listening_addresses = YamlReader::get_set<types::Address>(yml, LISTENING_ADDRESSES_TAG, version);
    }

    // Optional connection addresses
    if (YamlReader::is_tag_present(yml, CONNECTION_ADDRESSES_TAG))
    {
        object.connection_addresses = YamlReader::get_set<types::Address>(
            yml,
            CONNECTION_ADDRESSES_TAG,
            version);
    }

    // Optional TLS
    if (YamlReader::is_tag_present(yml, TLS_TAG))
    {
        object.tls_configuration = YamlReader::get<types::security::TlsConfiguration>(yml, TLS_TAG, version);
    }

    // Optional Repeater
    if (YamlReader::is_tag_present(yml, IS_REPEATER_TAG))
    {
        object.is_repeater = YamlReader::get<bool>(yml, IS_REPEATER_TAG, version);
    }
}

template <>
configuration::InitialPeersParticipantConfiguration YamlReader::get(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    configuration::InitialPeersParticipantConfiguration object;
    fill<configuration::InitialPeersParticipantConfiguration>(object, yml, version);
    return object;
}

/***************************
 * DDS ROUTER CONFIGURATION *
 ****************************/

template <>
std::shared_ptr<core::configuration::ParticipantConfiguration>
YamlReader::get<std::shared_ptr<core::configuration::ParticipantConfiguration>>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    // Kind required
    types::ParticipantKind kind = YamlReader::get<types::ParticipantKind>(yml, PARTICIPANT_KIND_TAG, version);

    logInfo(DDSROUTER_YAML_CONFIGURATION, "Loading Participant of kind " << kind << ".");

    switch (kind)
    {
        case types::ParticipantKind::blank:
        case types::ParticipantKind::dummy:
            return std::make_shared<core::configuration::ParticipantConfiguration>(
                YamlReader::get<core::configuration::ParticipantConfiguration>(yml, version));

        case types::ParticipantKind::echo:
            return std::make_shared<core::configuration::EchoParticipantConfiguration>(
                YamlReader::get<core::configuration::EchoParticipantConfiguration>(yml, version));

        case types::ParticipantKind::simple_rtps:
            return std::make_shared<core::configuration::SimpleParticipantConfiguration>(
                YamlReader::get<core::configuration::SimpleParticipantConfiguration>(yml, version));

        case types::ParticipantKind::local_discovery_server:
        case types::ParticipantKind::wan_discovery_server:
            return std::make_shared<core::configuration::DiscoveryServerParticipantConfiguration>(
                YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, version));

        case types::ParticipantKind::wan_initial_peers:
            return std::make_shared<core::configuration::InitialPeersParticipantConfiguration>(
                YamlReader::get<core::configuration::InitialPeersParticipantConfiguration>(yml, version));

        default:
            throw utils::ConfigurationException(
                      utils::Formatter() << "Unkown or non valid Participant kind:" << kind << ".");
            break;
    }
}

void _fill_ddsrouter_configuration_v1(
        core::configuration::DDSRouterConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    /////
    // Get optional allowlist
    if (YamlReader::is_tag_present(yml, ALLOWLIST_TAG))
    {
        object.allowlist = utils::convert_set_to_shared<types::FilterTopic, types::WildcardTopic>(
            YamlReader::get_set<types::WildcardTopic>(yml, ALLOWLIST_TAG, version));
    }

    /////
    // Get optional blocklist
    if (YamlReader::is_tag_present(yml, BLOCKLIST_TAG))
    {
        object.blocklist = utils::convert_set_to_shared<types::FilterTopic, types::WildcardTopic>(
            YamlReader::get_set<types::WildcardTopic>(yml, BLOCKLIST_TAG, version));
    }

    /////
    // Get builtin topics from allowlist
    for (const std::shared_ptr<FilterTopic>& topic : object.allowlist)
    {
        if (RealTopic::is_real_topic(topic->topic_name(), topic->topic_type()))
        {
            object.builtin_topics.emplace(
                std::make_shared<types::RealTopic>(
                    topic->topic_name(),
                    topic->topic_type(),
                    topic->topic_with_key()));
        }
    }

    /////
    // Get participants configurations from this yaml level
    for (Yaml::const_iterator participant_it = yml.begin();
            participant_it != yml.end();
            ++participant_it)
    {
        std::string name_str = participant_it->first.as<std::string>();

        // Check if it is not a key word
        if (is_tag(name_str))
        {
            continue;
        }

        // WORKAROUND: the name is not in the participant, as it is the key
        // In order to avoid creating a new ParticipantConfiguration get method for this version,
        // hack the participant info so it is read by latest versions
        Yaml participant_yml = participant_it->second;

        // Check the participant is correct
        // It must be a map
        if (!participant_yml.IsMap())
        {
            throw utils::ConfigurationException("Each Participant block must be a map.");
        }
        // It must have "type" tag
        if (!participant_yml[PARTICIPANT_KIND_TAG_V1])
        {
            throw utils::ConfigurationException(
                      STR_ENTRY <<
                          "Each Participant block must have <" << PARTICIPANT_KIND_TAG_V1 <<
                          "> tag with Participant Kind.");
        }

        // Set name
        participant_yml[PARTICIPANT_NAME_TAG] = name_str;
        // Set kind from type tag
        participant_yml[PARTICIPANT_KIND_TAG] = participant_yml[PARTICIPANT_KIND_TAG_V1];

        // Add new Participant with its configuration
        object.participants_configurations.insert(
            YamlReader::get<std::shared_ptr<core::configuration::ParticipantConfiguration>>(
                participant_yml,
                version));
    }
}

void _fill_ddsrouter_configuration_latest(
        core::configuration::DDSRouterConfiguration& object,
        const Yaml& yml,
        const YamlReaderVersion version)
{
    /////
    // Get optional allowlist
    if (YamlReader::is_tag_present(yml, ALLOWLIST_TAG))
    {
        object.allowlist = utils::convert_set_to_shared<types::FilterTopic, types::WildcardTopic>(
            YamlReader::get_set<types::WildcardTopic>(yml, ALLOWLIST_TAG, version));
    }

    /////
    // Get optional blocklist
    if (YamlReader::is_tag_present(yml, BLOCKLIST_TAG))
    {
        object.blocklist = utils::convert_set_to_shared<types::FilterTopic, types::WildcardTopic>(
            YamlReader::get_set<types::WildcardTopic>(yml, BLOCKLIST_TAG, version));
    }

    /////
    // Get optional builtin topics
    if (YamlReader::is_tag_present(yml, BUILTIN_TAG))
    {
        object.builtin_topics = utils::convert_set_to_shared<types::RealTopic, types::RealTopic>(
            YamlReader::get_set<types::RealTopic>(yml, BUILTIN_TAG, version));
    }

    /////
    // Get participants configurations. Required field, if get_value_in_tag fail propagate exception.
    auto participants_configurations_yml = YamlReader::get_value_in_tag(yml, COLLECTION_PARTICIPANTS_TAG);

    // TODO do it in a single instruction
    // Check it is a list
    if (!participants_configurations_yml.IsSequence())
    {
        throw utils::ConfigurationException(
                  utils::Formatter() <<
                      "Participant configurations must be specified in an array under tag: " <<
                      COLLECTION_PARTICIPANTS_TAG);
    }

    for (auto conf : participants_configurations_yml)
    {
        object.participants_configurations.insert(
            YamlReader::get<std::shared_ptr<core::configuration::ParticipantConfiguration>>(conf, version));
    }

    /////
    // Get optional number of threads
    if (YamlReader::is_tag_present(yml, NUMBER_THREADS_TAG))
    {
        object.number_of_threads = YamlReader::get<unsigned int>(yml, NUMBER_THREADS_TAG, version);
    }

}

template <>
core::configuration::DDSRouterConfiguration YamlReader::get<core::configuration::DDSRouterConfiguration>(
        const Yaml& yml,
        const YamlReaderVersion version)
{
    core::configuration::DDSRouterConfiguration object;
    switch (version)
    {
        case V_1_0:
            _fill_ddsrouter_configuration_v1(object, yml, version);
            break;

        default:
            _fill_ddsrouter_configuration_latest(object, yml, version);
            break;
    }
    return object;
}

std::ostream& operator <<(
        std::ostream& os,
        const YamlReaderVersion& version)
{
    switch (version)
    {
        case V_1_0:
            os << VERSION_TAG_V_1_0;
            break;

        case V_2_0:
            os << VERSION_TAG_V_2_0;
            break;

        case V_3_0:
            os << VERSION_TAG_V_3_0;
            break;

        case LATEST:
            os << VERSION_TAG_V_3_0;
            break;

        default:
            utils::tsnh(STR_ENTRY << "Value of YamlReaderVersion out of enumeration.");
            break;
    }

    return os;
}

} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

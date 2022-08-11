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
 * @file yaml_configuration_tags.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTERYAML_YAMLCONFIGURATIONTAGS_HPP_
#define _DDSROUTERYAML_YAMLCONFIGURATIONTAGS_HPP_

#include <set>
#include <string>

namespace eprosima {
namespace ddsrouter {
namespace yaml {

//! Retrieve a set with every tag used in the configuration of the DDSRouter or the Participants
std::set<std::string> ddsrouter_tags() noexcept;

//! Whether a string is a tag / key word
bool is_tag(
        const std::string& str) noexcept;

// Yaml related tags
constexpr const char* VERSION_TAG("version");       //! Version to parse the yaml with
constexpr const char* VERSION_TAG_V_1_0("v1.0");    //! Version v1.0
constexpr const char* VERSION_TAG_V_2_0("v2.0");    //! Version v2.0
constexpr const char* VERSION_TAG_V_3_0("v3.0");    //! Version v3.0

// Topics related tags
constexpr const char* ALLOWLIST_TAG("allowlist");     //! List of allowed topics
constexpr const char* BLOCKLIST_TAG("blocklist");     //! List of blocked topics
constexpr const char* BUILTIN_TAG("builtin-topics");  //! List of builtin topics
constexpr const char* TOPIC_NAME_TAG("name");         //! Name of a topic
constexpr const char* TOPIC_TYPE_NAME_TAG("type");    //! Type name of a topic
constexpr const char* TOPIC_KIND_TAG("keyed");        //! Kind of a topic (with or without key)
constexpr const char* TOPIC_RELIABLE_TAG("reliable"); //! The DataReaders of that topic will be configured as RELIABLE

constexpr const char* PARTICIPANT_KIND_TAG("kind");   //! Participant Kind
constexpr const char* PARTICIPANT_NAME_TAG("name");   //! Participant Name
constexpr const char* COLLECTION_PARTICIPANTS_TAG("participants"); //! TODO: add comment
constexpr const char* IS_REPEATER_TAG("repeater");   //! Is participant a repeater

// Echo related tags
constexpr const char* ECHO_DATA_TAG("data");            //! Echo Data received
constexpr const char* ECHO_DISCOVERY_TAG("discovery");  //! Echo Discovery received
constexpr const char* ECHO_VERBOSE_TAG("verbose");      //! Echo in verbose mode

// RTPS related tags
// Simple RTPS related tags
constexpr const char* DOMAIN_ID_TAG("domain"); //! Domain Id of the participant

// Discovery Server related tags
constexpr const char* DISCOVERY_SERVER_GUID_PREFIX_TAG("discovery-server-guid"); //! TODO: add comment
constexpr const char* LISTENING_ADDRESSES_TAG("listening-addresses"); //! TODO: add comment
constexpr const char* CONNECTION_ADDRESSES_TAG("connection-addresses"); //! TODO: add comment
constexpr const char* COLLECTION_ADDRESSES_TAG("addresses"); //! TODO: add comment

// TLS related tags
constexpr const char* TLS_TAG("tls"); //! TLS configuration tag
constexpr const char* TLS_CA_TAG("ca"); //! Certificate Authority Certificate
constexpr const char* TLS_PASSWORD_TAG("password"); //! DDS-Router Password
constexpr const char* TLS_PRIVATE_KEY_TAG("private_key"); //! DDS-Router Certificate Private Key
constexpr const char* TLS_CERT_TAG("cert"); //! DDS-Router Certificate
constexpr const char* TLS_DHPARAMS_TAG("dh_params"); //! Diffie-Hellman (DF) parameters

// Address related tags
constexpr const char* ADDRESS_IP_TAG("ip"); //! TODO: add comment
constexpr const char* ADDRESS_DNS_TAG("domain"); //! TODO: add comment
constexpr const char* ADDRESS_PORT_TAG("port"); //! TODO: add comment

constexpr const char* ADDRESS_IP_VERSION_TAG("ip-version"); //! TODO: add comment
constexpr const char* ADDRESS_IP_VERSION_V4_TAG("v4"); //! TODO: add comment
constexpr const char* ADDRESS_IP_VERSION_V6_TAG("v6"); //! TODO: add comment

constexpr const char* ADDRESS_TRANSPORT_TAG("transport"); //! TODO: add comment
constexpr const char* ADDRESS_TRANSPORT_UDP_TAG("udp"); //! TODO: add comment
constexpr const char* ADDRESS_TRANSPORT_TCP_TAG("tcp"); //! TODO: add comment

// Discovery Server Guid related tags
constexpr const char* DISCOVERY_SERVER_GUID_TAG("guid"); //! TODO: add comment
constexpr const char* DISCOVERY_SERVER_ID_TAG("id"); //! TODO: add comment
constexpr const char* DISCOVERY_SERVER_ID_ROS_TAG("ros-discovery-server"); //! TODO: add comment

// Advance configuration
constexpr const char* NUMBER_THREADS_TAG("threads"); //! Number of threads to configure the thread pool

// Old versions tags
constexpr const char* PARTICIPANT_KIND_TAG_V1("type"); //! Participant Kind


} /* namespace yaml */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERYAML_YAMLCONFIGURATIONTAGS_HPP_ */

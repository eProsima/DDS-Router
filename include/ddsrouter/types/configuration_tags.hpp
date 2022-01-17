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
 * @file configuration_tags.hpp
 *
 * This file contains constant values common for the whole project
 */

#ifndef _DDSROUTER_TYPES_CONFIGURATIONTAGS_HPP_
#define _DDSROUTER_TYPES_CONFIGURATIONTAGS_HPP_

namespace eprosima {
namespace ddsrouter {

//! Retrieve a set with every tag used in the configuration of the DDSRouter or the Participants
std::set<std::string> ddsrouter_tags() noexcept;

// Topics related tags
constexpr const char* ALLOWLIST_TAG("allowlist");   //! List of allowed topics
constexpr const char* BLOCKLIST_TAG("blocklist");   //! List of blocked topics
constexpr const char* TOPIC_NAME_TAG("name");       //! Name of a topic
constexpr const char* TOPIC_TYPE_NAME_TAG("type");  //! Type name of a topic
constexpr const char* TOPIC_KIND_TAG("keyed");      //! Kind of a topic (with or without key)

constexpr const char* PARTICIPANT_TYPE_TAG("type"); //! Participant Type

// RTPS related tags
// Simple RTPS related tags
constexpr const char* DOMAIN_ID_TAG("domain"); //! Domain Id of the participant

// Discovery Server related tags
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

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_CONFIGURATIONTAGS_HPP_ */

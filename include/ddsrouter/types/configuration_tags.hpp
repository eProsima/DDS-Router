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

constexpr const char* PARTICIPANT_TYPE_TAG("type"); //! Participant Type

// RTPS related tags
// Simple RTPS related tags
constexpr const char* DOMAIN_ID_TAG("domain"); //! Domain Id of the participant

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_CONFIGURATIONTAGS_HPP_ */

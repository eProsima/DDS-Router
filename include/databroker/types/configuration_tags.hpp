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

#ifndef _DATABROKER_TYPES_CONFIGURATIONTAGS_HPP_
#define _DATABROKER_TYPES_CONFIGURATIONTAGS_HPP_

namespace eprosima {
namespace databroker {

//! Retrieve a set with every tag used in the configuration of the Databroker or the Participants
std::set<std::string> databroker_tags() noexcept;

// Topics related tags
constexpr const char* WHITELIST_TAG("whitelist");
constexpr const char* BLACKLIST_TAG("blacklist");
constexpr const char* TOPIC_NAME_TAG("name");
constexpr const char* TOPIC_TYPE_NAME_TAG("type");

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TYPES_CONFIGURATIONTAGS_HPP_ */

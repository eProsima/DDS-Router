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
 * @file Guid.cpp
 *
 */

#include <limits.h>

#include <ddsrouter/types/endpoint/Guid.hpp>

namespace eprosima {
namespace ddsrouter {

const char* Guid::SERVER_DEFAULT_GUID_("01.0f.ff.00.00.00.00.00.00.00.00.ff");
const char* Guid::ROS_DISCOVERY_SERVER_GUID_("44.53.00.5f.45.50.52.4f.53.49.4d.41");

Guid::Guid (const std::string& str_guid)
{
    std::stringstream ss(str_guid);
    ss >> *this;
}

Guid::Guid (uint32_t seed)
{
    // TODO : extend it to the entity id
    guidPrefix = get_guid_prefix_from_seed(seed);
}

Guid& Guid::operator = (const fastrtps::rtps::GUID_t& other) noexcept
{
    this->guidPrefix = other.guidPrefix;
    this->entityId = other.entityId;
    return *this;
}

bool Guid::is_valid() const noexcept
{
    return guidPrefix != eprosima::fastrtps::rtps::GuidPrefix_t::unknown() &&
           entityId != eprosima::fastrtps::rtps::EntityId_t::unknown();
}

GuidPrefix Guid::get_guid_prefix_from_seed(const std::string& str_guid_prefix) noexcept
{
    GuidPrefix prefix;
    std::stringstream ss(str_guid_prefix);
    ss >> prefix;
    return prefix;
}

GuidPrefix Guid::get_guid_prefix_from_seed(uint32_t seed) noexcept
{
    // TODO : extend it to the whole guid, so seed is not truncated to octet
    GuidPrefix prefix = get_guid_prefix_from_seed(ROS_DISCOVERY_SERVER_GUID_);
    prefix.value[10] = static_cast<fastrtps::rtps::octet>(seed);
    return prefix;
}

GuidPrefix Guid::ros_discovery_server_guid_prefix() noexcept
{
    return get_guid_prefix_from_seed(ROS_DISCOVERY_SERVER_GUID_);
}

} /* namespace ddsrouter */
} /* namespace eprosima */

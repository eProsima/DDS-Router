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
 * @file GuidPrefix.cpp
 *
 */

#include <fastdds/rtps/attributes/ServerAttributes.h>

#include <ddsrouter_core/types/dds/GuidPrefix.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

const char* GuidPrefix::SERVER_DEFAULT_GUID_PREFIX_STR_("01.0f.00.00.00.00.00.00.00.00.ca.fe");
const char* GuidPrefix::ROS_DISCOVERY_SERVER_GUID_PREFIX_STR_(fastdds::rtps::DEFAULT_ROS2_SERVER_GUIDPREFIX);

GuidPrefix::GuidPrefix (
        const GuidPrefix_t& guid_prefix) noexcept
    : GuidPrefix_t(guid_prefix)
{
}

GuidPrefix::GuidPrefix (
        const std::string& str_prefix)
{
    std::stringstream ss(str_prefix);
    ss >> *this;

    // TODO launch exception
}

GuidPrefix::GuidPrefix (
        const char* str_prefix)
    : GuidPrefix(std::string(str_prefix))
{
}

GuidPrefix::GuidPrefix (
        bool ros /*= false*/,
        uint32_t id /*= 0*/) noexcept
{
    if (ros)
    {
        std::stringstream ss(ROS_DISCOVERY_SERVER_GUID_PREFIX_STR_);
        ss >> *this;
    }
    else
    {
        std::stringstream ss(SERVER_DEFAULT_GUID_PREFIX_STR_);
        ss >> *this;
    }

    // Modify depending on the seed
    // TODO : make available to modify for the whole guid prefix, so it is not truncated to 255
    value[2] = static_cast<fastrtps::rtps::octet>(id);
}

GuidPrefix::GuidPrefix (
        uint32_t id) noexcept
    : GuidPrefix(false, id)
{
}

GuidPrefix& GuidPrefix::operator = (
        const fastrtps::rtps::GuidPrefix_t& other) noexcept
{
    for (unsigned int i = 0; i < this->size; ++i)
    {
        this->value[i] = other.value[i];
    }
    return *this;
}

bool GuidPrefix::is_valid() const noexcept
{
    return *this != eprosima::fastrtps::rtps::GuidPrefix_t::unknown();
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

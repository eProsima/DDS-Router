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
 * @file Guid.hpp
 */

#ifndef _DDSROUTER_TYPES_GUID_HPP_
#define _DDSROUTER_TYPES_GUID_HPP_

#include <fastrtps/rtps/common/Guid.h>

namespace eprosima {
namespace ddsrouter {

//! Unique Id of every Endpoint
class Guid : public fastrtps::rtps::GUID_t
{
public:

    //! Using parent constructors
    using fastrtps::rtps::GUID_t::GUID_t;

    Guid (const std::string& str_guid);

    Guid& operator = (const fastrtps::rtps::GUID_t& other) noexcept;

    /**
     * Whether the guid is a valid one
     *
     * To be valid, the GuidPrefix and the EntityId must not be invalid / unknown
     */
    bool is_valid() const noexcept;

    static bool is_valid_str_guid(const std::string& str_guid) noexcept;

    static Guid get_guid_from_seed(uint32_t seed) noexcept;

    static Guid ros_discovery_server_guid() noexcept;

protected:

    static const char* SERVER_DEFAULT_GUID_; // 01.0f.ff.00.00.00.00.00.00.00.00.ff
    static const char* ROS_DISCOVERY_SERVER_GUID_; // 44.53.00.5f.45.50.52.4f.53.49.4d.41
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_GUID_HPP_ */

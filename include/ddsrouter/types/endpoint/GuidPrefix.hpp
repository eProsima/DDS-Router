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

#ifndef _DDSROUTER_TYPES_GUIDPREFIX_HPP_
#define _DDSROUTER_TYPES_GUIDPREFIX_HPP_

#include <fastdds/rtps/common/GuidPrefix_t.hpp>

namespace eprosima {
namespace ddsrouter {

//! TODO: comment
class GuidPrefix : public fastrtps::rtps::GuidPrefix_t
{
public:

    //! Using parent constructors
    using fastrtps::rtps::GuidPrefix_t::GuidPrefix_t;

    GuidPrefix (const GuidPrefix_t& guid_prefix) noexcept;

    GuidPrefix (const std::string& str_prefix);

    GuidPrefix (bool ros = false, uint32_t id = 0) noexcept;

    GuidPrefix (uint32_t id = 0) noexcept;

    GuidPrefix& operator = (const fastrtps::rtps::GuidPrefix_t& other) noexcept;

    /**
     * Whether the guid prefix is a valid one
     *
     * To be valid, the GuidPrefix must not be unknown
     */
    bool is_valid() const noexcept;

protected:

    static const char* SERVER_DEFAULT_GUID_PREFIX_STR_; // 01.0f.ff.00.00.00.00.00.00.00.00.ff
    static const char* ROS_DISCOVERY_SERVER_GUID_PREFIX_STR_; // 44.53.00.5f.45.50.52.4f.53.49.4d.41
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_GUIDPREFIX_HPP_ */

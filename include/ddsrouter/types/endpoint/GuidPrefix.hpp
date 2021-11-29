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
 * @file GuidPrefix.hpp
 */

#ifndef _DDSROUTER_TYPES_GUIDPREFIX_HPP_
#define _DDSROUTER_TYPES_GUIDPREFIX_HPP_

#include <fastdds/rtps/common/GuidPrefix_t.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * @brief GuidPrefix specific class for DDSRouter
 *
 * This class extends the \c GuidPrefix_t class in FastDDS, which is a RTPS Participant Unique Id.
 * This class does not belong to class \c Guid due to a design flaw in OOP (parallel hierarchy is the future).
 */
class GuidPrefix : public fastrtps::rtps::GuidPrefix_t
{
public:

    //! Using parent constructors
    using fastrtps::rtps::GuidPrefix_t::GuidPrefix_t;

    //! Constructor from Parent class
    GuidPrefix (
            const GuidPrefix_t& guid_prefix) noexcept;

    /**
     * @brief Construct a new Guid Prefix object from a string
     *
     * Format required: "xx.x...xx" with 12 values.
     * Non set zeros is allowed.
     * Lower case and Capital case are allowed for hexadecimal alphabetic numbers.
     *
     * @param guid_prefix string with the Guid Prefix
     */
    GuidPrefix (
            const std::string& str_prefix);

    /**
     * @brief Construct a new Guid Prefix object by seed
     *
     * Use \c id as seed to create an arbitrary \c GuidPrefix.
     * It could be used as default Guid the ROS2 Discovery Server Guid [44.53.00.5f.45.50.52.4f.53.49.4d.41]
     * or the DDSRouter Discovery Server Guid [01.0f.00.00.00.00.00.00.00.00.ca.fe].
     *
     * The value in 3rd position ([2]) is the one set by seed.
     *
     * @todo use the seed of \c id to modify the whole guid and not only one of the 12 values.
     *
     * @param ros : whether to use the Discovery Server ROS2 specific guid [Default: false]
     * @param id : number to seed for the final Guid Prefix [Default: 0]
     */
    GuidPrefix (
            bool ros = false,
            uint32_t id = 0) noexcept;

    /**
     * @brief Uses Router default Guid without using ROS Discovery Server
     *
     * @param id : number to seed for the final Guid Prefix [Default: 0]
     */
    GuidPrefix (
            uint32_t id = 0) noexcept;

    GuidPrefix& operator = (
            const fastrtps::rtps::GuidPrefix_t& other) noexcept;

    /**
     * Whether the guid prefix is a valid one
     *
     * To be valid, the GuidPrefix must not be unknown
     */
    bool is_valid() const noexcept;

    /////
    // YAML methods
    GuidPrefix(
        const RawConfiguration& configuration);
    RawConfiguration dump() const; // TODO: Once implemented add noexcept

protected:

    //! Default DDSRouter Discovery Server Guid Prefix
    static const char* SERVER_DEFAULT_GUID_PREFIX_STR_; // 01.0f.00.00.00.00.00.00.00.00.ca.fe

    //! Default ROS2 Discovery Server Guid Prefix
    static const char* ROS_DISCOVERY_SERVER_GUID_PREFIX_STR_; // 44.53.00.5f.45.50.52.4f.53.49.4d.41
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_GUIDPREFIX_HPP_ */

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
 * @file SpecificWriterQoS.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_ENDPOINT_SPECIFICWRITERQOS_HPP_
#define _DDSROUTERCORE_TYPES_ENDPOINT_SPECIFICWRITERQOS_HPP_

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/rtps/common/InstanceHandle.h>
#include <fastdds/rtps/common/Types.h>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! Partitions
using PartitionQosPolicy = eprosima::fastdds::dds::PartitionQosPolicy;

//! Ownership Strength
using OwnershipStrengthQosPolicy = eprosima::fastdds::dds::OwnershipStrengthQosPolicy;

/**
 * Collection of attributes of an Endpoint
 */
struct DDSROUTER_CORE_DllAPI SpecificWriterQoS
{
    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default SpecificWriterQoS with reader less restrictive parameters
    SpecificWriterQoS() = default;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    bool operator< (const SpecificWriterQoS& other) const noexcept;

    bool operator== (const SpecificWriterQoS& other) const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Partitions of the data
    PartitionQosPolicy partitions{};

    //! Ownership strength of the data
    OwnershipStrengthQosPolicy ownership_strength{};
};

/**
 * @brief \c PartitionQosPolicy to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const PartitionQosPolicy& qos);

/**
 * @brief \c OwnershipStrengthQosPolicy to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const OwnershipStrengthQosPolicy& qos);

/**
 * @brief \c SpecificWriterQoS to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const SpecificWriterQoS& qos);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_ENDPOINT_SPECIFICWRITERQOS_HPP_ */

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
 * @file QoS.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_ENDPOINT_QOS_HPP_
#define _DDSROUTERCORE_TYPES_ENDPOINT_QOS_HPP_

#include <fastdds/rtps/common/Types.h>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! Durability kind enumeration
using DurabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t;

//! Reliability kind enumeration
using ReliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t;

/**
 * Collection of attributes of an Endpoint
 */
struct DDSROUTER_CORE_DllAPI QoS
{
    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default QoS with reader less restrictive parameters
    QoS() noexcept = default;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    // OPERATOR OVERLOAD
    bool operator ==(
            const QoS& other) const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Durability kind (Default = VOLATILE)
    DurabilityKind durability_qos = DurabilityKind::VOLATILE;

    //! Reliability kind (Default = BEST_EFFORT)
    ReliabilityKind reliability_qos = ReliabilityKind::BEST_EFFORT;
};

/**
 * @brief \c DurabilityKind to stream serialization
 */
std::ostream& operator <<(
        std::ostream& os,
        const DurabilityKind& kind);

/**
 * @brief \c ReliabilityKind to stream serialization
 */
std::ostream& operator <<(
        std::ostream& os,
        const ReliabilityKind& kind);

/**
 * @brief \c QoS to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const QoS& qos);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_ENDPOINT_QOS_HPP_ */

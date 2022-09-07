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
 * @file TopicQoS.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_DDS_QOS_HPP_
#define _DDSROUTERCORE_TYPES_DDS_QOS_HPP_

#include <fastdds/dds/core/policy/QosPolicies.hpp>
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

//! History kind enumeration
using HistoryDepthType = unsigned int;

//! Partition configuration
using OwnershipQosPolicyKind = eprosima::fastdds::dds::OwnershipQosPolicyKind;

/**
 * Collection of attributes of an Endpoint
 */
struct DDSROUTER_CORE_DllAPI TopicQoS
{
    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default TopicQoS with reader less restrictive parameters
    TopicQoS() = default;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    // OPERATOR OVERLOAD
    bool operator ==(
            const TopicQoS& other) const noexcept;

    /////////////////////////
    // AUXILIARY METHODS
    /////////////////////////

    bool is_reliable() const noexcept;

    bool is_transient_local() const noexcept;

    bool has_ownership() const noexcept;

    bool has_partitions() const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Durability kind (Default = VOLATILE)
    DurabilityKind durability_qos = DurabilityKind::VOLATILE;

    //! Reliability kind (Default = BEST_EFFORT)
    ReliabilityKind reliability_qos = ReliabilityKind::BEST_EFFORT;

    //! Ownership kind of the topic
    OwnershipQosPolicyKind ownership_qos = OwnershipQosPolicyKind::SHARED_OWNERSHIP_QOS;

    //! Wether the topics uses partitions
    bool use_partitions = false;

    /**
     * @brief History Qos
     *
     * @note It only stores the depth because in router it will always be keep last, as RTPS has not resource limits.
     */
    HistoryDepthType history_depth = 1;
};

/**
 * @brief \c DurabilityKind to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const DurabilityKind& kind);

/**
 * @brief \c ReliabilityKind to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const ReliabilityKind& kind);

/**
 * @brief \c OwnershipQosPolicyKind to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const OwnershipQosPolicyKind& qos);

/**
 * @brief \c TopicQoS to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const TopicQoS& qos);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_QOS_HPP_ */

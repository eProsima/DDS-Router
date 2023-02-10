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
// limitations under the License\.

#pragma once

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/rtps/common/Types.h>

#include <ddspipe_core/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

//! Durability kind enumeration
using DurabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t;

//! Reliability kind enumeration
using ReliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t;

//! History kind enumeration
using HistoryDepthType = unsigned int;

//! Ownership configuration
using OwnershipQosPolicyKind = eprosima::fastdds::dds::OwnershipQosPolicyKind;

/**
 * Collection of QoS related with a Topic.
 *
 * The QoS associated with Topic are:
 * - Reliability
 * - Durability
 * - Ownership
 * - Partitions
 * - History Depth (history kind is always KEEP_LAST)
 *
 * @warning partitions are considered as a QoS, thus a Topic can only have partitions, or not have any, but cannot
 * support empty partition and partitions.
 *
 * @todo add keys to Topic QoS
 */
struct DDSPIPE_CORE_DllAPI TopicQoS
{
    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default TopicQoS with reader less restrictive parameters
    TopicQoS();

    /////////////////////////
    // OPERATORS
    /////////////////////////

    //! Equality operator
    bool operator ==(
            const TopicQoS& other) const noexcept;

    /////////////////////////
    // AUXILIARY METHODS
    /////////////////////////

    //! Whether the Topic is RELIABLE, not BEST_EFFORT
    bool is_reliable() const noexcept;

    //! Whether the Topic is TRANSIENT_LOCAL, not VOLATILE
    bool is_transient_local() const noexcept;

    //! Whether the Topic has EXCLUSIVE_OWNERSHIP, not SHARED_OWNERSHIP
    bool has_ownership() const noexcept;

    //! Whether the Topic has partitions, not empty partition
    bool has_partitions() const noexcept;

    /////////////////////////
    // GLOBAL VARIABLES
    /////////////////////////

    /**
     * @brief Global value to store the default history depth in this execution.
     *
     * This value can change along the execution.
     * Every new TopicQoS object will use this value as \c history_depth default.
     */
    static std::atomic<HistoryDepthType> default_history_depth;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Durability kind (Default = VOLATILE)
    DurabilityKind durability_qos = DurabilityKind::VOLATILE;

    //! Reliability kind (Default = BEST_EFFORT)
    ReliabilityKind reliability_qos = ReliabilityKind::BEST_EFFORT;

    //! Ownership kind of the topic
    OwnershipQosPolicyKind ownership_qos = OwnershipQosPolicyKind::SHARED_OWNERSHIP_QOS;

    //! Whether the topics uses partitions
    bool use_partitions = false;

    /**
     * @brief History Qos
     *
     * @note Default value would be taken from \c default_history_depth in object creation.
     * @note It only stores the depth because in router it will always be keep last, as RTPS has not resource limits.
     */
    HistoryDepthType history_depth = 5000;

    //! Whether the topic has key or not
    bool keyed = false;
};

/**
 * @brief \c DurabilityKind to stream serialization
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const DurabilityKind& kind);

/**
 * @brief \c ReliabilityKind to stream serialization
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const ReliabilityKind& kind);

/**
 * @brief \c OwnershipQosPolicyKind to stream serialization
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const OwnershipQosPolicyKind& qos);

/**
 * @brief \c TopicQoS to stream serialization
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const TopicQoS& qos);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

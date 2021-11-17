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

#ifndef _DDSROUTER_TYPES_QOS_HPP_
#define _DDSROUTER_TYPES_QOS_HPP_

#include <fastdds/rtps/common/Types.h>

namespace eprosima {
namespace ddsrouter {

//! Durability kind enumeration
using DurabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t;

//! Reliability kind enumeration
using ReliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t;

/**
 * Collection of attributes of an Endpoint
 */
class QoS
{
public:

    //! Default QoS with reader less restrictive parameters
    QoS();

    /**
     * Constructor of QoS class by its variables
     *
     * @param durability: durability kind
     * @param reliability: reliability kind
     */
    QoS(
            DurabilityKind durability,
            ReliabilityKind reliability);

    //! Whether this QoS is set with reliability
    DurabilityKind durability() const;

    //! Whether this QoS is set with durability
    ReliabilityKind reliability() const;

    // OPERATOR OVERLOAD
    bool operator ==(
            const QoS& other) const;

protected:

    //! Durability kind
    DurabilityKind durability_;

    //! Reliability kind
    ReliabilityKind reliability_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_QOS_HPP_ */

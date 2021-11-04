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

#ifndef _DATABROKER_TYPES_QOS_HPP_
#define _DATABROKER_TYPES_QOS_HPP_

#include <fastdds/rtps/common/Types.h>

namespace eprosima {
namespace databroker {

//! Durability kind enumeration
using DurabilityKind = eprosima::fastrtps::rtps::DurabilityKind_t;

//! Reliability kind enumeration
using ReliabilityKind = eprosima::fastrtps::rtps::ReliabilityKind_t;

/**
 * Collection of attributes of an Endpoint
 *
 * Only Durability <volatile | transient_local> are allowed
 * Only Reliability <reliable | best_effort> are allowed
 */
class QoS
{
public:

    /**
     * Constructor of QoS class by its variables
     *
     * @param durability: durability kind
     * @param reliability: reliability kind
     */
    QoS(
        DurabilityKind durability,
        ReliabilityKind reliability);

    /**
     * Constructor of QoS class by the QoS accepted possibilities
     *
     * @param is_volatile: whether the endpoint is volatile
     * @param is_reliable: whether the endpoint is reliable
     */
    QoS(
        bool is_volatile,
        bool is_reliable);

    //! Whether this QoS is set with reliability
    bool is_reliable() const;

    //! Whether this QoS is set with durability
    bool is_volatile() const;

protected:
    //! Durability kind
    DurabilityKind durability;

    //! Reliability kind
    ReliabilityKind reliability;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TYPES_QOS_HPP_ */

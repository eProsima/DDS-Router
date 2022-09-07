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
 * @file DataQoS.cpp
 *
 */

#include <ddsrouter_core/types/dds/DataQoS.hpp>
#include <ddsrouter_utils/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

std::ostream& operator <<(
        std::ostream& os,
        const PartitionQosPolicy& qos)
{
    os << "Partitions{";
    for (auto const& p : qos)
    {
        os << p.name() << ";";
    }
    os << "}";
    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const OwnershipStrengthQosPolicy& qos)
{
    os << "OwnershipStrength{" << qos.value << "}";
    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const DataQoS& qos)
{
    os <<
        "DataQoS{" << qos.partitions <<
        ";" << qos.ownership_strength <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

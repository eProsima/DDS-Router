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
 * @file SpecificEndpointQoS.cpp
 *
 */

#include <ddspipe_core/types/dds/SpecificEndpointQoS.hpp>
#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

bool SpecificEndpointQoS::operator < (
        const SpecificEndpointQoS& other) const noexcept
{
    // Ownership
    if (this->ownership_strength.value < other.ownership_strength.value)
    {
        return true;
    }
    else if (this->ownership_strength.value > other.ownership_strength.value)
    {
        return false;
    }

    // NOTE: PartitionQosPolicy operator< out of class implementation, this should be in Fast DDS file.
    if (this->partitions.size() < other.partitions.size())
    {
        return true;
    }
    else if (this->partitions.size() > other.partitions.size())
    {
        return false;
    }

    auto const this_names = this->partitions.getNames();
    auto const other_names = other.partitions.getNames();

    for (unsigned int i = 0; i < this_names.size(); ++i)
    {
        if (this_names[i] < other_names[i])
        {
            return true;
        }
        else if (this_names[i] > other_names[i])
        {
            return false;
        }
    }

    return false;
}

bool SpecificEndpointQoS::operator == (
        const SpecificEndpointQoS& other) const noexcept
{
    return this->partitions == other.partitions && this->ownership_strength == other.ownership_strength;
}

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
        const SpecificEndpointQoS& qos)
{
    os <<
        "SpecificEndpointQoS{" << qos.partitions <<
        ";" << qos.ownership_strength <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

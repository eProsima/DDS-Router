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
 * @file QoS.cpp
 *
 */

#include <databroker/types/endpoint/QoS.hpp>

namespace eprosima {
namespace databroker {

QoS::QoS(
        DurabilityKind durability,
        ReliabilityKind reliability)
    : durability_(durability)
    , reliability_(reliability)
{
}

DurabilityKind QoS::durability() const
{
    return durability_;
}

ReliabilityKind QoS::reliability() const
{
    return reliability_;
}

bool QoS::operator ==(
        const QoS& other) const
{
    return durability_ == other.durability_ && reliability_ == other.reliability_;
}

} /* namespace databroker */
} /* namespace eprosima */

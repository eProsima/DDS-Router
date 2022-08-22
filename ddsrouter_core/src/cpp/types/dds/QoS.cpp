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

#include <ddsrouter_core/types/dds/QoS.hpp>
#include <ddsrouter_utils/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

QoS::QoS() noexcept
    : durability_(DurabilityKind::VOLATILE)
    , reliability_(ReliabilityKind::BEST_EFFORT)
{
}

QoS::QoS(
        DurabilityKind durability,
        ReliabilityKind reliability) noexcept
    : durability_(durability)
    , reliability_(reliability)
{
}

DurabilityKind QoS::durability() const noexcept
{
    return durability_;
}

ReliabilityKind QoS::reliability() const noexcept
{
    return reliability_;
}

bool QoS::operator ==(
        const QoS& other) const noexcept
{
    return durability_ == other.durability_ && reliability_ == other.reliability_;
}

std::ostream& operator <<(
        std::ostream& os,
        const DurabilityKind& kind)
{
    std::string durability_str;
    switch (kind)
    {
        case DurabilityKind::VOLATILE:
            durability_str = "VOLATILE";
            break;

        case DurabilityKind::TRANSIENT_LOCAL:
            durability_str = "TRANSIENT_LOCAL";
            break;

        case DurabilityKind::TRANSIENT:
            durability_str = "TRANSIENT";
            break;

        case DurabilityKind::PERSISTENT:
            durability_str = "PERSISTENT";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Durability Kind.");
            break;
    }

    return os << durability_str;
}

std::ostream& operator <<(
        std::ostream& os,
        const ReliabilityKind& kind)
{
    std::string reliability_str;
    switch (kind)
    {
        case ReliabilityKind::RELIABLE:
            reliability_str = "RELIABLE";
            break;

        case ReliabilityKind::BEST_EFFORT:
            reliability_str = "BEST_EFFORT";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Reliability Kind.");
            break;
    }

    return os << reliability_str;
}

std::ostream& operator <<(
        std::ostream& os,
        const QoS& qos)
{
    os << "QoS{" << qos.durability_ << ";" << qos.reliability_ << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

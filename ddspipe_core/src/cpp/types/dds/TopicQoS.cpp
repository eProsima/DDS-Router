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
 * @file TopicQoS.cpp
 *
 */

#include <ddspipe_core/types/dds/TopicQoS.hpp>
#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

std::atomic<HistoryDepthType> TopicQoS::default_history_depth{1000};

TopicQoS::TopicQoS()
{
    // Set history by default
    history_depth = default_history_depth;
}

bool TopicQoS::operator ==(
        const TopicQoS& other) const noexcept
{
    return
        this->reliability_qos == other.reliability_qos &&
        this->durability_qos == other.durability_qos &&
        this->history_depth == other.history_depth &&
        this->ownership_qos == other.ownership_qos &&
        this->use_partitions == other.use_partitions;
}

bool TopicQoS::is_reliable() const noexcept
{
    return reliability_qos == ReliabilityKind::RELIABLE;
}

bool TopicQoS::is_transient_local() const noexcept
{
    return durability_qos == DurabilityKind::TRANSIENT_LOCAL;
}

bool TopicQoS::has_ownership() const noexcept
{
    return ownership_qos == OwnershipQosPolicyKind::EXCLUSIVE_OWNERSHIP_QOS;
}

bool TopicQoS::has_partitions() const noexcept
{
    return use_partitions;
}

std::ostream& operator <<(
        std::ostream& os,
        const DurabilityKind& kind)
{
    switch (kind)
    {
        case DurabilityKind::VOLATILE:
            os << "VOLATILE";
            break;

        case DurabilityKind::TRANSIENT_LOCAL:
            os << "TRANSIENT_LOCAL";
            break;

        case DurabilityKind::TRANSIENT:
            os << "TRANSIENT";
            break;

        case DurabilityKind::PERSISTENT:
            os << "PERSISTENT";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Durability Kind.");
            break;
    }

    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const ReliabilityKind& kind)
{
    switch (kind)
    {
        case ReliabilityKind::RELIABLE:
            os << "RELIABLE";
            break;

        case ReliabilityKind::BEST_EFFORT:
            os << "BEST_EFFORT";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Reliability Kind.");
            break;
    }

    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const OwnershipQosPolicyKind& kind)
{
    switch (kind)
    {
        case OwnershipQosPolicyKind::SHARED_OWNERSHIP_QOS:
            os << "SHARED";
            break;

        case OwnershipQosPolicyKind::EXCLUSIVE_OWNERSHIP_QOS:
            os << "EXCLUSIVE";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Ownership Kind.");
            break;
    }

    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const TopicQoS& qos)
{
    os <<
        "TopicQoS{" << qos.durability_qos <<
        ";" << qos.reliability_qos <<
        ";" << qos.ownership_qos <<
        (qos.has_partitions() ? ";partitions" : "") <<
        ";depth(" << qos.history_depth << ")" <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

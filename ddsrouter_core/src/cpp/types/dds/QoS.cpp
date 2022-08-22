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

bool QoS::operator ==(
        const QoS& other) const noexcept
{
    return
        durability_qos == other.durability_qos &&
        reliability_qos == other.reliability_qos &&
        history_qos == other.history_qos &&
        partition_qos == other.partition_qos;
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
        const HistoryQosPolicy& qos)
{
    switch (qos.kind)
    {
        case eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS:
            os << "KEEP_LAST(" << qos.depth << ")";
            break;

        case eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS:
            os << "KEEP_ALL";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Reliability Kind.");
            break;
    }

    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const PartitionQosPolicy& qos)
{
    os << "Partitions{";
    for(auto& partition : qos)
    {
        os << partition.name() << ";";
    }
    os << "}";

    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const QoS& qos)
{
    os <<
        "QoS{" << qos.durability_qos <<
        ";" << qos.reliability_qos <<
        ";" << qos.history_qos <<
        ";" << qos.partition_qos <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

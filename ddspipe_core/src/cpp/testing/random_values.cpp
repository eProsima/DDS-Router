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

#include <ddspipe_core/testing/random_values.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace testing {

using namespace eprosima::ddspipe::core::types;

Guid random_guid(
        unsigned int seed /* = 1 */)
{
    Guid guid;
    guid.entityId.value[3] = static_cast<eprosima::fastrtps::rtps::octet>(seed);
    guid.guidPrefix.value[0] = 0x01;
    guid.guidPrefix.value[1] = 0x0f;
    return guid;
}

DomainId random_domain(
        unsigned int seed /* = 0 */)
{
    return DomainId(static_cast<DomainIdType>(seed));
}

GuidPrefix random_guid_prefix(
        unsigned int seed /* = 0 */,
        bool ros /* = false */)
{
    if (ros)
    {
        return GuidPrefix(true, seed);
    }
    else
    {
        return GuidPrefix(static_cast<uint32_t>(seed));
    }
}

ParticipantId random_participant_id(
        unsigned int seed /* = 0 */)
{
    std::vector<std::string> names = {
        "participant",
        "PART_1",
        "echo",
        "Barro_p",
    };

    return ParticipantId(names[seed % names.size()] + std::to_string(seed));
}

DdsTopic random_dds_topic(
        unsigned int seed /* = 0 */)
{
    DdsTopic topic;
    topic.m_topic_name = "TopicName_" + std::to_string(seed);
    topic.type_name = "TopicType_" + std::to_string(seed);
    topic.topic_qos = random_topic_qos(seed);
    return topic;
}

EndpointKind random_endpoint_kind(
        unsigned int seed /* = 0 */)
{
    if (seed % 2)
    {
        return EndpointKind::reader;
    }
    else
    {
        return EndpointKind::writer;
    }
}

Endpoint random_endpoint(
        unsigned int seed /* = 0 */)
{
    Endpoint endpoint;
    endpoint.active = (seed % 2);
    endpoint.guid = random_guid(seed);
    endpoint.discoverer_participant_id = random_participant_id(seed);
    endpoint.kind = random_endpoint_kind(seed);
    endpoint.topic = random_dds_topic(seed);
    return endpoint;
}

TopicQoS random_topic_qos(
        unsigned int seed /* = 0 */)
{
    TopicQoS qos;

    if (seed % 2)
    {
        qos.reliability_qos = ReliabilityKind::BEST_EFFORT;
    }
    else
    {
        qos.reliability_qos = ReliabilityKind::RELIABLE;
    }

    switch ((seed / 2) % 4)
    {
        case 0:
            qos.durability_qos = DurabilityKind::VOLATILE;
            break;

        case 1:
            qos.durability_qos = DurabilityKind::TRANSIENT_LOCAL;
            break;

        case 2:
            qos.durability_qos = DurabilityKind::TRANSIENT;
            break;

        case 3:
            qos.durability_qos = DurabilityKind::PERSISTENT;
            break;

        default:
            break;
    }

    return qos;
}

} /* namespace testing */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

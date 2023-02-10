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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_core/types/endpoint/Endpoint.hpp>
#include <ddspipe_core/types/dds/TopicQoS.hpp>
#include <ddspipe_core/types/dds/Guid.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

// Get a random TopicQoS configuration
TopicQoS random_qos(
        uint16_t seed = 0)
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

// Get a random topic name
DistributedTopic random_topic(
        uint16_t seed = 0)
{
    return DistributedTopic("TopicName_" + std::to_string(seed), "TopicType_" + std::to_string(seed), false, random_qos(seed));
}

// Get a random topic name
EndpointKind random_endpoint_kind(
        uint16_t seed = 0)
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

// Get a random guid
Guid random_valid_guid(
        uint16_t seed = 0)
{
    eprosima::fastrtps::rtps::GuidPrefix_t guid_prefix;
    std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d." + std::to_string(seed)) >> guid_prefix;
    return Guid(
        guid_prefix,
        seed);
}

/**
 * Test \c Endpoint constructor
 */
TEST(EndpointTest, constructor)
{
    EndpointKind kind;
    Guid guid;
    DistributedTopic topic = random_topic();
    // TODO: add data qos

    Endpoint endpoint(kind, guid, topic);

    ASSERT_EQ(endpoint.kind(), kind);
    ASSERT_EQ(endpoint.guid(), guid);
    ASSERT_EQ(endpoint.topic_qos(), topic.topic_qos.get_reference());
    ASSERT_EQ(endpoint.topic(), topic);
}

/**
 * Test \c Endpoint \c kind getter method
 *
 * CASES:
 *  Writer
 *  Reader
 */
TEST(EndpointTest, kind_getter)
{
    Guid guid;
    DistributedTopic topic = random_topic();

    // Writer
    {
        Endpoint endpoint(EndpointKind::writer, guid, topic);
        ASSERT_EQ(endpoint.kind(), EndpointKind::writer);
    }

    // Reader
    {
        Endpoint endpoint(EndpointKind::reader, guid, topic);
        ASSERT_EQ(endpoint.kind(), EndpointKind::reader);
    }
}

/**
 * Test \c Endpoint \c guid getter method
 *
 * CASES:
 *  Default guid
 *  Random guids
 */
TEST(EndpointTest, guid_getter)
{
    DistributedTopic topic = random_topic();
    EndpointKind kind = random_endpoint_kind();

    // Default guid
    {
        Guid guid;
        Endpoint endpoint(kind, guid, topic);
        ASSERT_EQ(endpoint.guid(), guid);
    }

    // Random guids
    {
        for (uint16_t i = 0; i < 10; i++)
        {
            Guid guid = random_valid_guid(i);
            Endpoint endpoint(kind, guid, topic);
            ASSERT_EQ(endpoint.guid(), random_valid_guid(i)) << i;
        }
    }
}

/**
 * Test \c Endpoint \c qos getter method
 *
 * CASES:
 *  Random TopicQoS
 */
TEST(EndpointTest, qos_getter)
{
    Guid guid = random_valid_guid();
    DistributedTopic topic = random_topic();
    EndpointKind kind = random_endpoint_kind();

    // Random guids
    {
        for (uint16_t i = 0; i < 8; i++)
        {
            Endpoint endpoint(kind, guid, topic);
            ASSERT_EQ(topic.topic_qos, endpoint.topic_qos()) << i;
        }
    }
}

/**
 * Test \c Endpoint \c topic getter method
 *
 * CASES:
 *  Random Topics
 */
TEST(EndpointTest, topic_getter)
{
    Guid guid = random_valid_guid();
    EndpointKind kind = random_endpoint_kind();

    // Random guids
    {
        for (uint16_t i = 0; i < 10u; i++)
        {
            DistributedTopic topic = random_topic(i);
            Endpoint endpoint(kind, guid, topic);
            ASSERT_EQ(endpoint.topic(), random_topic(i)) << i;
        }
    }
}

/**
 * Test \c Endpoint \c active getter method
 *
 * CASES:
 *  Default value
 *  Change to invalid
 *  Change to invalid and to valid
 */
TEST(EndpointTest, active_getter)
{
    Guid guid = random_valid_guid();
    DistributedTopic topic = random_topic();
    EndpointKind kind = random_endpoint_kind();

    // Default value
    {
        Endpoint endpoint(kind, guid, topic);
        ASSERT_TRUE(endpoint.active());
    }

    // Change to invalid
    {
        Endpoint endpoint(kind, guid, topic);
        endpoint.active(false);
        ASSERT_FALSE(endpoint.active());
    }

    // Change to invalid and to valid
    {
        Endpoint endpoint(kind, guid, topic);
        endpoint.active(false);
        endpoint.active(true);
        ASSERT_TRUE(endpoint.active());
    }
}

/**
 * Test \c Endpoint \c active setter method
 *
 * Same test as active_setter
 * CASES:
 *  Default value
 *  Change to invalid
 *  Change to invalid and to valid
 */
TEST(EndpointTest, active_setter)
{
    Guid guid = random_valid_guid();
    DistributedTopic topic = random_topic();
    EndpointKind kind = random_endpoint_kind();

    // Default value
    {
        Endpoint endpoint(kind, guid, topic);
        ASSERT_TRUE(endpoint.active());
    }

    // Change to invalid
    {
        Endpoint endpoint(kind, guid, topic);
        endpoint.active(false);
        ASSERT_FALSE(endpoint.active());
    }

    // Change to invalid and to valid
    {
        Endpoint endpoint(kind, guid, topic);
        endpoint.active(false);
        endpoint.active(true);
        ASSERT_TRUE(endpoint.active());
    }
}

/**
 * Test \c Endpoint \c is_writer getter method
 *
 * CASES:
 *  Writer
 *  Reader
 */
TEST(EndpointTest, is_writer)
{
    Guid guid;
    DistributedTopic topic = random_topic();

    // Writer
    {
        Endpoint endpoint(EndpointKind::writer, guid, topic);
        ASSERT_TRUE(endpoint.is_writer());
    }

    // Reader
    {
        Endpoint endpoint(EndpointKind::reader, guid, topic);
        ASSERT_FALSE(endpoint.is_writer());
    }
}

/**
 * Test \c Endpoint \c is_reader getter method
 *
 * CASES:
 *  Writer
 *  Reader
 */
TEST(EndpointTest, is_reader)
{
    Guid guid;
    DistributedTopic topic = random_topic();

    // Writer
    {
        Endpoint endpoint(EndpointKind::writer, guid, topic);
        ASSERT_FALSE(endpoint.is_reader());
    }

    // Reader
    {
        Endpoint endpoint(EndpointKind::reader, guid, topic);
        ASSERT_TRUE(endpoint.is_reader());
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

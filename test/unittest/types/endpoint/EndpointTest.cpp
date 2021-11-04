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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <databroker/types/endpoint/Endpoint.hpp>
#include <databroker/types/endpoint/QoS.hpp>
#include <databroker/types/endpoint/Guid.hpp>

using namespace eprosima::databroker;

// Get a random QoS configuration
QoS random_qos(uint seed = 0)
{
    DurabilityKind durability;
    ReliabilityKind reliability;

    if (seed % 2)
    {
        reliability = ReliabilityKind::BEST_EFFORT;
    }
    else
    {
        reliability = ReliabilityKind::RELIABLE;
    }

    switch ((seed/2) % 4)
    {
    case 0:
        durability = DurabilityKind::VOLATILE;
        break;

    case 1:
        durability = DurabilityKind::TRANSIENT_LOCAL;
        break;

    case 2:
        durability = DurabilityKind::TRANSIENT;
        break;

    case 3:
        durability = DurabilityKind::PERSISTENT;
        break;

    default:
        break;
    }

    return QoS(durability, reliability);
}

// Get a random topic name
RealTopic random_topic(uint seed = 0)
{
    return RealTopic("TopicName_" + std::to_string(seed), "TopicType_" + std::to_string(seed));
}

// Get a random guid
Guid random_valid_guid(uint seed = 0)
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
    QoS qos = random_qos();
    RealTopic topic = random_topic();

    Endpoint(kind, guid, qos, topic);
}

/**
 * Test \c Endpoint \c kind getter method
 */
TEST(EndpointTest, kind_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c guid getter method
 */
TEST(EndpointTest, guid_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c qos getter method
 */
TEST(EndpointTest, qos_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c topic getter method
 */
TEST(EndpointTest, topic_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c active getter method
 */
TEST(EndpointTest, active_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c active setter method
 */
TEST(EndpointTest, active_setter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c is_writer getter method
 */
TEST(EndpointTest, is_writer)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c Endpoint \c is_reader getter method
 */
TEST(EndpointTest, is_reader)
{
    // TODO
    ASSERT_TRUE(false);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

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
#include <test_utils.hpp>

#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/exception/InconsistencyException.hpp>
#include <ddsrouter/types/endpoint/Endpoint.hpp>
#include <ddsrouter/types/endpoint/Guid.hpp>
#include <ddsrouter/types/endpoint/QoS.hpp>
#include <ddsrouter/types/ReturnCode.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>

using namespace eprosima::ddsrouter;

/**
 * Test \c DiscoveryDatabase \c topic_exists method
 *
 * CASES:
 *  Topic not present in database
 *  Topic present in database
 */
TEST(DiscoveryDatabaseTest, topic_exists)
{
    DiscoveryDatabase discovery_database;
    Guid guid_1 = test::random_guid(1);
    Guid guid_2 = test::random_guid(2);
    QoS qos;
    RealTopic topic("test", "test");
    Endpoint endpoint_1(EndpointKind::READER, guid_1, qos, topic);
    Endpoint endpoint_2(EndpointKind::READER, guid_2, qos, topic);

    ASSERT_FALSE(discovery_database.topic_exists(topic));
    discovery_database.add_endpoint(endpoint_1);
    ASSERT_TRUE(discovery_database.topic_exists(topic));
    discovery_database.add_endpoint(endpoint_2);
    ASSERT_TRUE(discovery_database.topic_exists(topic));
    discovery_database.erase_endpoint(guid_1);
    ASSERT_TRUE(discovery_database.topic_exists(topic));
    discovery_database.erase_endpoint(guid_2);
    ASSERT_FALSE(discovery_database.topic_exists(topic));
}

/**
 * Test \c DiscoveryDatabase \c endpoint_exists method
 *
 * CASES:
 *  Endpoint not present in database
 *  Endpoint present in database
 */
TEST(DiscoveryDatabaseTest, endpoint_exists)
{
    DiscoveryDatabase discovery_database;
    Guid guid;
    QoS qos;
    RealTopic topic("test", "test");
    Endpoint endpoint(EndpointKind::READER, guid, qos, topic);

    ASSERT_FALSE(discovery_database.endpoint_exists(guid));
    discovery_database.add_endpoint(endpoint);
    ASSERT_TRUE(discovery_database.endpoint_exists(guid));
    discovery_database.erase_endpoint(guid);
    ASSERT_FALSE(discovery_database.endpoint_exists(guid));
}

/**
 * Test \c DiscoveryDatabase \c add_endpoint method
 *
 * CASES:
 *  Endpoint to insert not present
 *  Endpoint to insert already present and active
 *  Endpoint to insert already present but inactive
 */
TEST(DiscoveryDatabaseTest, add_endpoint)
{
    DiscoveryDatabase discovery_database;
    QoS qos;
    RealTopic topic("original", "original");
    // Create active endpoint
    Guid active_guid = test::random_guid(1);
    Endpoint active_endpoint(EndpointKind::READER, active_guid, qos, topic);
    // Create inactive endpoint
    Guid inactive_guid = test::random_guid(2);
    Endpoint inactive_endpoint(EndpointKind::READER, inactive_guid, qos, topic);
    inactive_endpoint.active(false);

    // Insert endpoints
    ASSERT_TRUE(discovery_database.add_endpoint(active_endpoint));
    ASSERT_EQ(discovery_database.get_endpoint(active_guid), active_endpoint);
    ASSERT_TRUE(discovery_database.add_endpoint(inactive_endpoint));
    ASSERT_EQ(discovery_database.get_endpoint(inactive_guid), inactive_endpoint);

    // Add new endpoint with same guid as already stored active endpoint
    // Should throw \c InconsistencyException
    RealTopic new_topic("new", "new");
    Endpoint active_new_endpoint(EndpointKind::READER, active_guid, qos, new_topic);
    ASSERT_THROW(discovery_database.add_endpoint(active_new_endpoint), InconsistencyException);

    // Add new endpoint with same guid as already stored inactive endpoint
    // Should update entry
    Endpoint inactive_new_endpoint(EndpointKind::READER, inactive_guid, qos, new_topic);
    ASSERT_TRUE(discovery_database.add_endpoint(inactive_new_endpoint));
    ASSERT_EQ(discovery_database.get_endpoint(inactive_guid), inactive_new_endpoint);
}

/**
 * Test \c DiscoveryDatabase \c update_endpoint method
 *
 * CASES:
 *  Endpoint to be updated not present in database
 *  Endpoint to be updated present in database
 */
TEST(DiscoveryDatabaseTest, update_endpoint)
{
    DiscoveryDatabase discovery_database;
    Guid guid = test::random_guid(1);
    QoS qos;
    RealTopic topic("original", "original");
    Endpoint endpoint(EndpointKind::READER, guid, qos, topic);
    RealTopic new_topic("new", "new");
    Endpoint new_endpoint(EndpointKind::READER, guid, qos, new_topic);
    // Endpoint to be updated not yet inserted
    ASSERT_THROW(discovery_database.update_endpoint(new_endpoint), InconsistencyException);

    // Insert endpoint
    ASSERT_TRUE(discovery_database.add_endpoint(endpoint));
    ASSERT_TRUE(discovery_database.topic_exists(topic));
    ASSERT_FALSE(discovery_database.topic_exists(new_topic));
    ASSERT_EQ(discovery_database.get_endpoint(guid), endpoint);

    // Update endpoint
    ASSERT_TRUE(discovery_database.update_endpoint(new_endpoint));
    ASSERT_FALSE(discovery_database.topic_exists(topic));
    ASSERT_TRUE(discovery_database.topic_exists(new_topic));
    ASSERT_EQ(discovery_database.get_endpoint(guid), new_endpoint);
}

/**
 * Test \c DiscoveryDatabase \c erase_endpoint method
 *
 * CASES:
 *  Endpoint to erase not present in database
 *  Endpoint to erase present in database
 */
TEST(DiscoveryDatabaseTest, erase_endpoint)
{
    DiscoveryDatabase discovery_database;
    Guid guid = test::random_guid(1);
    QoS qos;
    RealTopic topic("test", "test");
    Endpoint endpoint(EndpointKind::READER, guid, qos, topic);

    // Endpoint to erase not yet inserted
    ASSERT_THROW(discovery_database.erase_endpoint(guid), InconsistencyException);

    // Insert endpoint
    discovery_database.add_endpoint(endpoint);
    ASSERT_TRUE(discovery_database.endpoint_exists(guid));

    // Erase endpoint
    ASSERT_EQ(discovery_database.erase_endpoint(guid), ReturnCode::RETCODE_OK);
    ASSERT_FALSE(discovery_database.endpoint_exists(guid));
}

/**
 * Test \c DiscoveryDatabase \c get_endpoint method
 *
 * CASES:
 *  Endpoint to retrieve not present in database
 *  Endpoint to retrieve present in database
 */
TEST(DiscoveryDatabaseTest, get_endpoint)
{
    DiscoveryDatabase discovery_database;
    Guid guid = test::random_guid(1);
    QoS qos;
    RealTopic topic("test", "test");
    Endpoint endpoint(EndpointKind::READER, guid, qos, topic);

    // Try to fetch a not stored endpoint
    ASSERT_FALSE(discovery_database.endpoint_exists(guid));
    ASSERT_THROW(discovery_database.get_endpoint(guid).kind(), InconsistencyException);

    // Insert and get endpoint
    discovery_database.add_endpoint(endpoint);
    ASSERT_TRUE(discovery_database.endpoint_exists(guid));
    ASSERT_EQ(discovery_database.get_endpoint(guid), endpoint);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

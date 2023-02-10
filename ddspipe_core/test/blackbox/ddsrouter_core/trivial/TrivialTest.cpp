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

#include <algorithm>
#include <iostream>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>

#include <ddspipe_core/core/DdsRouter.hpp>
#include <ddspipe_core/participants/participant/auxiliar/DummyParticipant.hpp>
#include <cpp_utils/Log.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>

#include <cpp_utils/utils.hpp>

using namespace eprosima::ddsrouter::test;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

std::vector<PayloadUnit> random_payload(
        uint16_t seed = 1)
{
    std::vector<PayloadUnit> payload;

    for (int i = 0; i < seed; i++)
    {
        payload.push_back(i);
    }

    return payload;
}

configuration::DdsRouterConfiguration void_configuration()
{
    configuration::DdsRouterConfiguration configuration;
    configuration.participants_configurations =
    {
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId("ParticipantVoid1"),
            ParticipantKind::blank,
            false
            ),
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId("ParticipantVoid2"),
            ParticipantKind::blank,
            false
            )
    };

    return configuration;
}

/**
 * @brief Create a \c DdsRouterConfiguration with 2 dummy participants and one builtin topic
 *
 * @return configuration::DdsRouterConfiguration
 */
configuration::DdsRouterConfiguration simple_configuration(
        const std::string& participant_1_name = "Participant1",
        const std::string& participant_2_name = "Participant2",
        const std::string& topic_name = "topic_dummy",
        const std::string& topic_type = "type_dummy")
{
    configuration::DdsRouterConfiguration configuration;

    configuration.builtin_topics =
    {
        std::set<std::shared_ptr<DistributedTopic>>({std::make_shared<DistributedTopic>(topic_name, topic_type)}),
    };

    configuration.participants_configurations =
    {
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId(participant_1_name),
            ParticipantKind::dummy,
            false
            ),
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId(participant_2_name),
            ParticipantKind::dummy,
            false
            )
    };

    return configuration;
}

/**
 * Test Whole DdsRouter initialization by initializing two EmptyParticipants
 */
TEST(TrivialTest, trivial_void_initialization)
{
    // Create DdsRouter entity
    DdsRouter router(void_configuration());
    router.start();
    router.stop();

    // Let test finish without failing
}

/**
 * Test Whole DdsRouter initialization by initializing two DummyParticipants and a DistributedTopic
 */
TEST(TrivialTest, trivial_dummy_initialization)
{
    // Create DdsRouter entity
    DdsRouter router(simple_configuration());
    router.start();
    router.stop();

    // Let test finish without failing
}

/**
 * Test Whole DdsRouter interfaces by using two DummyParticipants and send one message from one to the other
 *
 * TODO
 */
TEST(TrivialTest, trivial_communication)
{
    DdsRouter router(simple_configuration());
    router.start();

    DummyParticipant* participant_1 = DummyParticipant::get_participant(ParticipantId("Participant1"));
    DummyParticipant* participant_2 = DummyParticipant::get_participant(ParticipantId("Participant2"));
    ASSERT_NE(participant_1, nullptr);
    ASSERT_NE(participant_2, nullptr);

    DistributedTopic topic("topic_dummy", "type_dummy");
    Guid guid = random_guid();
    std::vector<PayloadUnit> payload = random_payload(3);

    DummyDataReceived data;
    data.source_guid = guid;
    data.payload = payload;

    participant_1->simulate_data_reception(topic, data);

    // Sleep until first data arrives to the writer
    participant_2->wait_until_n_data_sent(topic, 1);

    std::vector<DummyDataStored> data_received = participant_2->get_data_that_should_have_been_sent(topic);

    ASSERT_EQ(data_received.size(), 1u);
    ASSERT_EQ(data_received[0].source_guid, guid);
    ASSERT_EQ(data_received[0].payload, payload);

    router.stop();
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

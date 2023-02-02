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

#include <cpp_utils/utils.hpp>

#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_core/participants/participant/auxiliar/DummyParticipant.hpp>
#include <ddsrouter_core/participants/participant/auxiliar/BlankParticipant.hpp>
#include <cpp_utils/Log.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

#include <test_utils.hpp>

using namespace eprosima::ddsrouter::participants;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

namespace test
{



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

WholeConfiguration void_configuration()
{
    WholeConfiguration configuration;

    configuration.participant_database->add_participant(
        ParticipantId("ParticipantVoid1"),
        std::make_shared<BlankParticipant>(
            ParticipantId("ParticipantVoid1"),
            false
        )
    );

    configuration.participant_database->add_participant(
        ParticipantId("ParticipantVoid2"),
        std::make_shared<BlankParticipant>(
            ParticipantId("ParticipantVoid2"),
            false
        )
    );

    return configuration;
}

/**
 * @brief Create a \c DDSRouterConfiguration with 2 dummy participants and one builtin topic
 *
 * @return configuration::DDSRouterConfiguration
 */
WholeConfiguration simple_configuration(
        const std::string& participant_1_name = "Participant1",
        const std::string& participant_2_name = "Participant2",
        const std::string& topic_name = "topic_dummy",
        const std::string& topic_type = "type_dummy")
{
    WholeConfiguration configuration;

    configuration.configuration.builtin_topics.insert(std::make_shared<DdsTopic>(topic_name, topic_type));

    configuration.participant_database->add_participant(
        ParticipantId(participant_1_name),
        std::make_shared<DummyParticipant>(
            ParticipantConfiguration(
                ParticipantId(participant_1_name),
                false),
            configuration.payload_pool,
            configuration.discovery_database,
        ),
    );

    configuration.participant_database->add_participant(
        ParticipantId(participant_2_name),
        std::make_shared<DummyParticipant>(
            ParticipantConfiguration(
                ParticipantId(participant_2_name),
                false),
            configuration.payload_pool,
            configuration.discovery_database,
        ),
    );

    return configuration;
}

} // namespace test

/**
 * Test Whole DDSRouter initialization by initializing two EmptyParticipants
 */
TEST(TrivialTest, trivial_void_initialization)
{
    // Create DDSRouter entity
    auto configuration = void_configuration();
    DDSRouter router(
        ddsrouter_configuration.configuration,
        ddsrouter_configuration.discovery_database,
        ddsrouter_configuration.payload_pool,
        ddsrouter_configuration.participants_database);

    router.start();
    router.stop();

    // Let test finish without failing
}

/**
 * Test Whole DDSRouter initialization by initializing two DummyParticipants and a DdsTopic
 */
TEST(TrivialTest, trivial_dummy_initialization)
{
    // Create DDSRouter entity
    auto configuration = simple_configuration();
    DDSRouter router(
        ddsrouter_configuration.configuration,
        ddsrouter_configuration.discovery_database,
        ddsrouter_configuration.payload_pool,
        ddsrouter_configuration.participants_database);

    router.start();
    router.stop();

    // Let test finish without failing
}

/**
 * Test Whole DDSRouter interfaces by using two DummyParticipants and send one message from one to the other
 *
 * TODO
 */
TEST(TrivialTest, trivial_communication)
{
    auto configuration = simple_configuration();
    DDSRouter router(
        ddsrouter_configuration.configuration,
        ddsrouter_configuration.discovery_database,
        ddsrouter_configuration.payload_pool,
        ddsrouter_configuration.participants_database);

    router.start();

    DummyParticipant* participant_1 = configuration.participants_database.get(ParticipantId("Participant1")).get();
    DummyParticipant* participant_2 = configuration.participants_database.get(ParticipantId("Participant2")).get();
    ASSERT_NE(participant_1, nullptr);
    ASSERT_NE(participant_2, nullptr);

    DdsTopic topic("topic_dummy", "type_dummy");
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

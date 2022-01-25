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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>
#include <ddsrouter/types/utils.hpp>

using namespace eprosima::ddsrouter;

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

configuration::DDSRouterConfiguration void_configuration()
{
    return configuration::DDSRouterConfiguration(
        std::set<std::shared_ptr<FilterTopic>>(),
        std::set<std::shared_ptr<FilterTopic>>(),
        std::set<std::shared_ptr<RealTopic>>(),
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>>(
            {
                std::make_shared<configuration::ParticipantConfiguration>(
                    ParticipantId("ParticipantVoid1"),
                    ParticipantKind::VOID
                ),
                std::make_shared<configuration::ParticipantConfiguration>(
                    ParticipantId("ParticipantVoid2"),
                    ParticipantKind::VOID
                )
            }
        ));
}

/**
 * @brief Create a \c DDSRouterConfiguration with 2 dummy participants and one builtin topic
 *
 * @return configuration::DDSRouterConfiguration
 */
configuration::DDSRouterConfiguration simple_configuration(
    const std::string& participant_1_name = "Participant1",
    const std::string& participant_2_name = "Participant2",
    const std::string& topic_name = "topic_dummy",
    const std::string& topic_type = "type_dummy")
{
    return configuration::DDSRouterConfiguration(
        std::set<std::shared_ptr<FilterTopic>>(),
        std::set<std::shared_ptr<FilterTopic>>(),
        std::set<std::shared_ptr<RealTopic>>({std::make_shared<RealTopic>(topic_name, topic_type)}),
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>>(
            {
                std::make_shared<configuration::ParticipantConfiguration>(
                    ParticipantId(participant_1_name),
                    ParticipantKind::DUMMY
                ),
                std::make_shared<configuration::ParticipantConfiguration>(
                    ParticipantId(participant_2_name),
                    ParticipantKind::DUMMY
                )
            }
        ));
}

/**
 * Test Whole DDSRouter initialization by initializing two VoidParticipants
 */
TEST(TrivialTest, trivial_void_initialization)
{
    // Create DDSRouter entity
    DDSRouter router(void_configuration());
    router.start();
    router.stop();

    // Let test finish without failing
}

/**
 * Test Whole DDSRouter initialization by initializing two DummyParticipants and a RealTopic
 */
TEST(TrivialTest, trivial_dummy_initialization)
{
    // Create DDSRouter entity
    DDSRouter router(simple_configuration());
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
    DDSRouter router(simple_configuration());
    router.start();

    DummyParticipant* participant_1 = DummyParticipant::get_participant(ParticipantId("Participant1"));
    DummyParticipant* participant_2 = DummyParticipant::get_participant(ParticipantId("Participant2"));
    ASSERT_NE(participant_1, nullptr);
    ASSERT_NE(participant_2, nullptr);

    RealTopic topic("topic_dummy", "type_dummy");
    Guid guid = test::random_guid();
    std::vector<PayloadUnit> payload = random_payload(3);

    DummyDataReceived data;
    data.source_guid = guid;
    data.payload = payload;

    participant_1->simulate_data_reception(topic, data);

    // Sleep until first data arrives to the writer
    participant_2->wait_until_n_data_sent(topic, 1);

    std::vector<DummyDataStored> data_received = participant_2->get_data_that_should_have_been_sent(topic);

    ASSERT_EQ(1, data_received.size());
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

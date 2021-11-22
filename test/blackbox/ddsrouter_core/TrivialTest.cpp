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

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/participant/implementations/auxiliar/DummyParticipant.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>
#include <ddsrouter/types/utils.hpp>

using namespace eprosima::ddsrouter;

Guid random_guid(
        uint16_t seed = 1)
{
    Guid guid;
    guid.entityId.value[3] = seed;
    guid.guidPrefix.value[0] = 0x01;
    guid.guidPrefix.value[1] = 0x0f;
    return guid;
}

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

/**
 * Test Whole DDSRouter initialization by initializing two VoidParticipants
 */
TEST(TrivialTest, trivial_void_initialization)
{
    // Load configuration
    RawConfiguration router_configuration =
        load_configuration_from_file("resources/trivial_test_void_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);

    // Let test finish without failing
}

/**
 * Test Whole DDSRouter initialization by initializing two DummyParticipants and a RealTopic
 */
TEST(TrivialTest, trivial_dummy_initialization)
{
    // Load configuration
    RawConfiguration router_configuration =
        load_configuration_from_file("resources/trivial_test_dummy_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);

    // Let test finish without failing
}

/**
 * Test Whole DDSRouter interfaces by using two DummyParticipants and send one message from one to the other
 *
 * TODO
 */
TEST(TrivialTest, trivial_communication)
{
    // Load configuration
    RawConfiguration router_configuration =
        load_configuration_from_file("resources/trivial_test_dummy_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);
    router.start();

    DummyParticipant* participant_1 = DummyParticipant::get_participant(ParticipantId("participant_1"));
    DummyParticipant* participant_2 = DummyParticipant::get_participant(ParticipantId("participant_2"));
    RealTopic topic("trivial_topic", "trivial_type");
    Guid guid = random_guid();
    std::vector<PayloadUnit> payload = random_payload(3);

    DummyDataReceived data;
    data.source_guid = guid;
    data.payload = payload;

    participant_1->simulate_data_reception(topic, data);

    // It must sleep so it gives time for Router to redirect msg
    std::this_thread::sleep_for(std::chrono::seconds(2));

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
    // Activate log
    Log::SetVerbosity(Log::Kind::Info);
    Log::SetCategoryFilter(std::regex("(DDSROUTER)"));

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

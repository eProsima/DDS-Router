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
#include <chrono>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>
#include <TestLogHandler.hpp>

#include <ddsrouter_core/core/DDSRouter.hpp>
#include <participant/auxiliar/DummyParticipant.hpp>
#include <reader/auxiliar/DummyReader.hpp>
#include <writer/auxiliar/DummyWriter.hpp>
#include <core/DDSRouterImpl.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.ipp>
#include <ddsrouter_core/types/dds/Payload.hpp>
#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

using namespace eprosima::ddsrouter::utils;
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

configuration::DDSRouterConfiguration void_configuration()
{
    return configuration::DDSRouterConfiguration(
        TopicKeySet<FilterTopic>(),
        TopicKeySet<FilterTopic>(),
        TopicKeySet<RealTopic>(),
        ParticipantKeySet<std::shared_ptr<configuration::ParticipantConfiguration>>(
    {
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId("ParticipantVoid1", ParticipantKind::blank)
            ),
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId("ParticipantVoid2", ParticipantKind::blank)
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
        TopicKeySet<FilterTopic>(),
        TopicKeySet<FilterTopic>(),
        TopicKeySet<RealTopic>({RealTopic(topic_name, topic_type)}),
        ParticipantKeySet<std::shared_ptr<configuration::ParticipantConfiguration>>(
    {
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId(participant_1_name, ParticipantKind::dummy)
            ),
        std::make_shared<configuration::ParticipantConfiguration>(
            ParticipantId(participant_2_name, ParticipantKind::dummy)
            )
    }
            ));
}

class TestDDSRouter : public DDSRouterImpl
{
public:

    using DDSRouterImpl::DDSRouterImpl;

    IParticipant* get_participant(
            ParticipantName name)
    {
        auto found = participants_iterable_.find(name);

        if (found == std::end(participants_iterable_))
        {
            throw InconsistencyException("Participant not existing");
        }
        return *found;
    }

};

/**
 * Test Whole DDSRouter initialization by initializing two EmptyParticipants
 */
TEST(TrivialTest, trivial_void_initialization)
{
    // test::TestLogHandler test_log_handler(utils::Log::Kind::Info);
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
    // test::TestLogHandler test_log_handler(utils::Log::Kind::Info);
    // Create DDSRouter entity
    DDSRouter router(simple_configuration());
    router.start();
    router.stop();

    // Let test finish without failing
}

/**
 * Test Whole DDSRouter interfaces by using two DummyParticipants and send one message from one to the other
 *
 */
TEST(TrivialTest, trivial_communication)
{
    TestDDSRouter router(simple_configuration());

    router.start();

    auto reader_1 =
            static_cast<DummyParticipant*>(router.get_participant("Participant1"))->get_topic_reader(RealTopic(
                        "topic_dummy",
                        "type_dummy"));
    auto writer_1 =
            static_cast<DummyParticipant*>(router.get_participant("Participant1"))->get_topic_writer(RealTopic(
                        "topic_dummy",
                        "type_dummy"));
    auto reader_2 =
            static_cast<DummyParticipant*>(router.get_participant("Participant2"))->get_topic_reader(RealTopic(
                        "topic_dummy",
                        "type_dummy"));
    auto writer_2 =
            static_cast<DummyParticipant*>(router.get_participant("Participant2"))->get_topic_writer(RealTopic(
                        "topic_dummy",
                        "type_dummy"));

    reader_1->initialize(1);

    // Check readers initial values
    ASSERT_EQ(reader_1->get_enqueued(), 0);
    ASSERT_EQ(reader_1->get_notified(), 0);
    ASSERT_EQ(reader_1->get_forwarded(), 0);

    ASSERT_EQ(reader_2->get_enqueued(), 0);
    ASSERT_EQ(reader_2->get_notified(), 0);
    ASSERT_EQ(reader_2->get_forwarded(), 0);

    // Check writers initial values
    ASSERT_EQ(writer_1->get_received(), 0);
    ASSERT_EQ(writer_2->get_received(), 0);

    std::string sample_message("my-message");

    const auto total = 10000u;

    for (auto i = 0u; i < total; i++)
    {
        reader_1->mock_data_reception(sample_message);
    }

    while (reader_1->get_forwarded() < total)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Check readers counters are valid
    ASSERT_EQ(reader_1->get_enqueued(), 0);
    ASSERT_EQ(reader_1->get_notified(), total);
    ASSERT_EQ(reader_1->get_forwarded(), total);

    ASSERT_EQ(reader_2->get_enqueued(), 0);
    ASSERT_EQ(reader_2->get_notified(), 0);
    ASSERT_EQ(reader_2->get_forwarded(), 0);

    // Check writers counters are valid
    ASSERT_EQ(writer_1->get_received(), 0);

    ASSERT_EQ(writer_2->get_received(), total);
    ASSERT_TRUE(writer_2->check_content(sample_message));

    router.stop();
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

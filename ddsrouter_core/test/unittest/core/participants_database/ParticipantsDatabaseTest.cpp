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

#include <set>

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <cpp_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_core/core/ParticipantsDatabase.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/participants/participant/auxiliar/BlankParticipant.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/**
 * Test the addition of a participant to the database
 *
 * CASES:
 *
 *  Add not stored participant
 *  Add already stored participant
 */
TEST(ParticipantsDatabaseTest, add_participant)
{
    std::shared_ptr<ParticipantsDatabase> participants_database = std::make_shared<ParticipantsDatabase>();
    // Check database is empty
    ASSERT_TRUE(participants_database->empty());

    ParticipantId id("void_part");
    std::shared_ptr<participants::BlankParticipant> participant = std::make_shared<participants::BlankParticipant>(id);
    // Insert participant
    participants_database->add_participant(participant->id(), participant);

    // Reinsert and check it throws an error
    ASSERT_THROW(
        participants_database->add_participant(
            participant->id(),
            participant),
        eprosima::utils::InconsistencyException);
}

/**
 * Test \c ParticipantsDatabase \c empty method
 *
 * CASES:
 *  Empty database
 *  Non-empty database
 */
TEST(ParticipantsDatabaseTest, empty)
{
    std::shared_ptr<ParticipantsDatabase> participants_database = std::make_shared<ParticipantsDatabase>();
    // Empty database
    ASSERT_TRUE(participants_database->empty());

    // Non-empty database
    ParticipantId id("void_part");
    std::shared_ptr<participants::BlankParticipant> participant = std::make_shared<participants::BlankParticipant>(id);
    participants_database->add_participant(participant->id(), participant);
    ASSERT_FALSE(participants_database->empty());
}

/**
 * Test \c ParticipantsDatabase \c get_participant method
 *
 * CASES:
 *  Empty configuration
 *  Random configuration
 */
TEST(ParticipantsDatabaseTest, get_participant)
{
    std::shared_ptr<ParticipantsDatabase> participants_database = std::make_shared<ParticipantsDatabase>();

    ParticipantId id("void_part");
    std::shared_ptr<participants::BlankParticipant> participant = std::make_shared<participants::BlankParticipant>(id);
    participants_database->add_participant(participant->id(), participant);

    ASSERT_EQ(participants_database->get_participant(id), participant);
}

/**
 * Test \c ParticipantsDatabase \c get_participants_ids method
 */
TEST(ParticipantsDatabaseTest, get_participants_ids)
{
    std::shared_ptr<ParticipantsDatabase> participants_database = std::make_shared<ParticipantsDatabase>();

    ParticipantId id1("void_p1");
    std::shared_ptr<participants::BlankParticipant> participant1 = std::make_shared<participants::BlankParticipant>(id1);
    participants_database->add_participant(participant1->id(), participant1);

    ParticipantId id2("void_p2");
    std::shared_ptr<participants::BlankParticipant> participant2 = std::make_shared<participants::BlankParticipant>(id2);
    participants_database->add_participant(participant2->id(), participant2);

    std::set<ParticipantId> ids {id2, id1};
    ASSERT_EQ(ids, participants_database->get_participants_ids());
}

/**
 * Test \c ParticipantsDatabase \c get_participants_map getter method
 */
TEST(ParticipantsDatabaseTest, get_participants_map)
{
    ParticipantId id("void_part");
    std::shared_ptr<participants::BlankParticipant> participant = std::make_shared<participants::BlankParticipant>(id);
    std::map<ParticipantId, std::shared_ptr<IParticipant>> participants = {{id, participant}};

    std::shared_ptr<ParticipantsDatabase> participants_database = std::make_shared<ParticipantsDatabase>();
    participants_database->add_participant(id, participant);

    ASSERT_EQ(participants_database->get_participants_map(), participants);
}

/**
 * Test \c ParticipantsDatabase \c size getter method
 */
TEST(ParticipantsDatabaseTest, size)
{
    std::shared_ptr<ParticipantsDatabase> participants_database = std::make_shared<ParticipantsDatabase>();
    // Check database is empty
    ASSERT_EQ(participants_database->size(), 0u);

    ParticipantId id_1("void_part");
    std::shared_ptr<participants::BlankParticipant> participant_1 = std::make_shared<participants::BlankParticipant>(id_1);
    // Insert participant_1
    participants_database->add_participant(participant_1->id(), participant_1);
    ASSERT_EQ(participants_database->size(), 1u);

    ParticipantId id_2("simple_part");
    std::shared_ptr<participants::BlankParticipant> participant_2 = std::make_shared<participants::BlankParticipant>(id_2);
    // Insert participant_2
    participants_database->add_participant(participant_2->id(), participant_2);
    ASSERT_EQ(participants_database->size(), 2u);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter/exceptions/InconsistencyException.hpp>
#include <ddsrouter/participant/implementations/auxiliar/VoidParticipant.hpp>
#include <ddsrouter/participant/ParticipantsDatabase.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>

using namespace eprosima::ddsrouter;

namespace eprosima {
namespace ddsrouter {
namespace test {

/**
 * This class is used to expose protected methods of the parent class
 * so they can be tested.
 */
class ParticipantsDatabase : public eprosima::ddsrouter::ParticipantsDatabase
{
public:

    ParticipantsDatabase()
    {
    }

    ParticipantsDatabase(
            std::map<ParticipantId, std::shared_ptr<IParticipant>> participants)
        : ParticipantsDatabase()
    {
        participants_ = participants;
    }

    std::shared_ptr<IParticipant> pop(
            const ParticipantId& id) noexcept
    {
        return pop_(id);
    }

    std::shared_ptr<IParticipant> pop() noexcept
    {
        return pop_();
    }

    void add_participant(
            ParticipantId id,
            std::shared_ptr<IParticipant> participant,
            std::size_t expected_size)
    {
        add_participant_(id, participant);
        // Verify correct insertion
        auto it = participants_.find(id);
        ASSERT_EQ(it->second, participant);
        ASSERT_EQ(participants_.size(), expected_size);
    }

};

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

/*********************
* PROTECTED METHODS **
*********************/

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
    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>();
    // Check database is empty
    ASSERT_TRUE(participants_database->empty());

    ParticipantId id("void_part");
    std::shared_ptr<VoidParticipant> participant = std::make_shared<VoidParticipant>(id);
    // Insert participant
    participants_database->add_participant(participant->id(), participant, 1);

    // Reinsert and check it throws an error
    ASSERT_THROW(participants_database->add_participant(participant->id(), participant, 1), InconsistencyException);
}

/**
 * Test the extraction of a participant from the database
 *
 * CASES:
 *  Empty database
 *  Pop random participant
 *  Pop specific participant
 *  Pop not stored participant
 *  Pop stored participant
 */
TEST(ParticipantsDatabaseTest, pop)
{
    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>();
    ASSERT_TRUE(participants_database->empty());
    // Empty database, no participant should be popped
    ASSERT_TRUE(participants_database->pop() == nullptr);

    ParticipantId id1("void_p1");
    std::shared_ptr<VoidParticipant> participant1 = std::make_shared<VoidParticipant>(id1);
    ParticipantId id2("void_p2");
    std::shared_ptr<VoidParticipant> participant2 = std::make_shared<VoidParticipant>(id2);
    // Pop not yet stored participant
    ASSERT_TRUE(participants_database->pop(id1) == nullptr);
    ASSERT_TRUE(participants_database->pop(id2) == nullptr);

    // Insert participants into database
    participants_database->add_participant(participant1->id(), participant1, 1);
    participants_database->add_participant(participant2->id(), participant2, 2);

    // Pop stored participants
    ASSERT_EQ(participants_database->pop(id1), participant1);
    ASSERT_EQ(participants_database->pop(), participant2);
    // Participants should have been removed
    ASSERT_TRUE(participants_database->get_participant(id1) == nullptr);
    ASSERT_TRUE(participants_database->get_participant(id2) == nullptr);
    ASSERT_TRUE(participants_database->empty());
}

/*********************
* PUBLIC METHODS **
*********************/

/**
 * Test \c ParticipantsDatabase \c empty method
 *
 * CASES:
 *  Empty database
 *  Non-empty database
 */
TEST(ParticipantsDatabaseTest, empty)
{
    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>();
    // Empty database
    ASSERT_TRUE(participants_database->empty());

    // Non-empty database
    ParticipantId id("void_part");
    std::shared_ptr<VoidParticipant> participant = std::make_shared<VoidParticipant>(id);
    participants_database->add_participant(participant->id(), participant, 1);
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
    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>();

    ParticipantId id("void_part");
    std::shared_ptr<VoidParticipant> participant = std::make_shared<VoidParticipant>(id);
    participants_database->add_participant(participant->id(), participant, 1);

    ASSERT_EQ(participants_database->get_participant(id), participant);
}

/**
 * Test \c ParticipantsDatabase \c get_participants_ids method
 */
TEST(ParticipantsDatabaseTest, get_participants_ids)
{
    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>();

    ParticipantId id1("void_p1");
    std::shared_ptr<VoidParticipant> participant1 = std::make_shared<VoidParticipant>(id1);
    participants_database->add_participant(participant1->id(), participant1, 1);

    ParticipantId id2("void_p2");
    std::shared_ptr<VoidParticipant> participant2 = std::make_shared<VoidParticipant>(id2);
    participants_database->add_participant(participant2->id(), participant2, 2);

    std::set<ParticipantId> ids {id2, id1};
    ASSERT_EQ(ids, participants_database->get_participants_ids());
}

/**
 * Test \c ParticipantsDatabase \c get_participants_map getter method
 */
TEST(ParticipantsDatabaseTest, get_participants_map)
{
    ParticipantId id("void_part");
    std::shared_ptr<VoidParticipant> participant = std::make_shared<VoidParticipant>(id);
    std::map<ParticipantId, std::shared_ptr<IParticipant>> participants = {{id, participant}};

    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>(
        participants);
    ASSERT_EQ(participants_database->get_participants_map(), participants);
}

/**
 * Test \c ParticipantsDatabase \c size getter method
 */
TEST(ParticipantsDatabaseTest, size)
{
    std::shared_ptr<test::ParticipantsDatabase> participants_database = std::make_shared<test::ParticipantsDatabase>();
    // Check database is empty
    ASSERT_EQ(participants_database->size(), 0);

    ParticipantId id_1("void_part");
    std::shared_ptr<VoidParticipant> participant_1 = std::make_shared<VoidParticipant>(id_1);
    // Insert participant_1
    participants_database->add_participant(participant_1->id(), participant_1, 1);
    ASSERT_EQ(participants_database->size(), 1);

    ParticipantId id_2("simple_part");
    std::shared_ptr<VoidParticipant> participant_2 = std::make_shared<VoidParticipant>(id_2);
    // Insert participant_2
    participants_database->add_participant(participant_2->id(), participant_2, 2);
    ASSERT_EQ(participants_database->size(), 2);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

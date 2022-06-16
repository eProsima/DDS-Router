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

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <participant/ParticipantsRegistry.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace test {

/**
 * This class is used to expose protected methods of the parent class
 * so they can be tested.
 */
class ParticipantsRegistry : protected eprosima::ddsrouter::core::ParticipantsRegistry
{
    using BaseT = eprosima::ddsrouter::core::ParticipantsRegistry;

public:

    ParticipantsRegistry() = default;

    IParticipant* add_participant(
            const ParticipantId& id)
    {
        return BaseT::add_participant(id);
    }

    bool has_participant(
            const ParticipantId& id)
    {
        return BaseT::get_participant(id.name()) != nullptr;
    }

    IParticipant* get_participant(
            const ParticipantId& id)
    {
        return BaseT::get_participant(id.name());
    }

    bool pop_participant(
            const ParticipantId& id)
    {
        auto poped = BaseT::pop_participant(id.name());
        return poped.get() != nullptr;
    }

};

} /* namespace test */
} /* namespace core */
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
TEST(ParticipantsRegistryTest, add_participant)
{
    test::ParticipantsRegistry participants_registry;


    ParticipantId id("void_part", ParticipantKind::blank);

    ASSERT_FALSE(participants_registry.has_participant(id));

    // Insert participant
    participants_registry.add_participant(id);

    ASSERT_TRUE(participants_registry.has_participant(id));

    ASSERT_NE(participants_registry.get_participant(id), nullptr);

    // Reinsert same-id participant and check it throws an error
    ASSERT_THROW( participants_registry.add_participant( id ), utils::InconsistencyException );
}

/**
 * Test the extraction of a participant from the database
 *
 * CASES:
 *  Pop not stored participant
 *  Pop specific participant
 */
TEST(ParticipantsRegistryTest, pop_participant)
{
    test::ParticipantsRegistry participants_registry;

    ParticipantId id1("void_p1", ParticipantKind::blank);
    ParticipantId id2("void_p2", ParticipantKind::blank);

    // Expected for has/pop when registry is empty
    ASSERT_FALSE(participants_registry.has_participant(id1));
    ASSERT_FALSE(participants_registry.has_participant(id2));
    ASSERT_FALSE(participants_registry.pop_participant(id1));
    ASSERT_FALSE(participants_registry.pop_participant(id2));

    // Insert participants into database
    participants_registry.add_participant(id1);
    participants_registry.add_participant(id2);

    ASSERT_TRUE(participants_registry.has_participant(id1));
    ASSERT_NE(participants_registry.get_participant(id1), nullptr);

    ASSERT_TRUE(participants_registry.has_participant(id2));
    ASSERT_NE(participants_registry.get_participant(id2), nullptr);

    // Pop of p1 succeds
    ASSERT_TRUE(participants_registry.pop_participant(id1));
    ASSERT_EQ(participants_registry.get_participant(id1), nullptr);
    // Second pop of p1 fails
    ASSERT_FALSE(participants_registry.pop_participant(id1));

    // Still has participant 2
    ASSERT_TRUE(participants_registry.has_participant(id2));
    ASSERT_NE(participants_registry.get_participant(id2), nullptr);

    // Pop of p2 succeds
    ASSERT_TRUE(participants_registry.pop_participant(id2));
    // Second pop of p2 fails
    ASSERT_FALSE(participants_registry.pop_participant(id2));
    ASSERT_EQ(participants_registry.get_participant(id2), nullptr);
}

/*********************
* PUBLIC METHODS **
*********************/

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

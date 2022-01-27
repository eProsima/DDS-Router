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

#include <ddsrouter/communication/payload_pool/MapPayloadPool.hpp>
#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/participant/ParticipantFactory.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>

using namespace eprosima::ddsrouter;

namespace eprosima {
namespace ddsrouter {
namespace test {

/*
 * Generate all required objects for participant creation,
 * and then create a participant from an id string
 */
std::shared_ptr<IParticipant> create_participant(
        ParticipantKind kind)
{
    ParticipantFactory participant_factory;
    std::shared_ptr<PayloadPool> payload_pool = std::make_shared<MapPayloadPool>();
    std::shared_ptr<DiscoveryDatabase> discovery_database = std::make_shared<DiscoveryDatabase>();
    std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration =
            random_participant_configuration(kind);

    return participant_factory.create_participant(participant_configuration, payload_pool, discovery_database);
}

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test \c ParticipantFactory \c create_participant method using each of the valid Participant kinds
 */
TEST(ParticipantFactoryTest, create_participant)
{
    // For each Participant Type
    for (ParticipantKind kind : ParticipantKind::all_valid_participant_kinds())
    {
        std::shared_ptr<IParticipant> void_participant = test::create_participant(kind);
        ASSERT_EQ(void_participant->kind(), kind) << "Failed in " << kind;
    }
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Invalid participant, should throw \c ConfigurationException
 */
TEST(ParticipantFactoryTest, create_invalid_participant)
{
    ASSERT_THROW(test::create_participant(ParticipantKind::PARTICIPANT_KIND_INVALID), ConfigurationException);
}

/**
 * Test \c ParticipantFactory \c remove_participant method
 * TODO: Test that actions performed in participant creation
 * are correctly undone by \c remove_participant
 */
TEST(ParticipantFactoryTest, remove_participant)
{
    // TODO
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

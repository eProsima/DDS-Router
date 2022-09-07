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

#include <efficiency/payload/MapPayloadPool.hpp>
#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <participant/IParticipant.hpp>
#include <core/ParticipantFactory.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

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
    discovery_database->enable();
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
    for (ParticipantKind kind : ALL_VALID_PARTICIPANT_KINDS)
    {
        std::shared_ptr<IParticipant> void_participant = test::create_participant(kind);
        ASSERT_EQ(void_participant->kind(), kind) << "Failed in " << kind;
    }
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Invalid participant, should throw \c utils::ConfigurationException
 */
TEST(ParticipantFactoryTest, create_invalid_participant)
{
    ASSERT_THROW(test::create_participant(ParticipantKind::invalid), utils::ConfigurationException);
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

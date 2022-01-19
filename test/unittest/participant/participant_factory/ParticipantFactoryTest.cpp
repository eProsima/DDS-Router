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

#include <ddsrouter/communication/PayloadPool.hpp>
#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/exception/ConfigurationException.hpp>
#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/participant/ParticipantFactory.hpp>
#include <ddsrouter/types/configuration_tags.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantKind.hpp>

using namespace eprosima::ddsrouter;

namespace eprosima {
namespace ddsrouter {
namespace test {

// TODO: refactor these tests so they test with type in id and with type in Configuration
// TODO: add these functions in a common utils test
void set_domain(
        RawConfiguration& configuration,
        uint16_t seed = 0)
{
    configuration[DOMAIN_ID_TAG] = seed;
}

ParticipantConfiguration get_random_participant_configuration(
        ParticipantId id,
        ParticipantKind type,
        uint32_t seed = 0)
{
    RawConfiguration config;

    switch (type())
    {
        case ParticipantKind::SIMPLE_RTPS:
            set_domain(config, seed);
            break;

        // Add configurations por Participant Types that require configuration arguments

        default:
            break;
    }

    return ParticipantConfiguration(id, config);
}

/*
 * Generate all required objects for participant creation,
 * and then create a participant from an id string
 */
std::shared_ptr<IParticipant> create_participant(
        std::string id_str,
        ParticipantKind type)
{
    ParticipantFactory participant_factory;
    std::shared_ptr<PayloadPool> payload_pool = std::make_shared<PayloadPool>();
    std::shared_ptr<DiscoveryDatabase> discovery_database = std::make_shared<DiscoveryDatabase>();
    ParticipantId id(id_str);
    ParticipantConfiguration participant_configuration =
            get_random_participant_configuration(id, type);

    return participant_factory.create_participant(participant_configuration, payload_pool, discovery_database);
}

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Void participant
 */
TEST(ParticipantFactoryTest, create_void_participant)
{
    std::shared_ptr<IParticipant> void_participant = test::create_participant("void", ParticipantKind::VOID);
    ASSERT_EQ(void_participant->type()(), ParticipantKind::VOID);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Echo participant
 */
TEST(ParticipantFactoryTest, create_echo_participant)
{
    std::shared_ptr<IParticipant> echo_participant = test::create_participant("echo", ParticipantKind::ECHO);
    ASSERT_EQ(echo_participant->type()(), ParticipantKind::ECHO);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Dummy participant
 */
TEST(ParticipantFactoryTest, create_dummy_participant)
{
    std::shared_ptr<IParticipant> dummy_participant = test::create_participant("dummy", ParticipantKind::DUMMY);
    ASSERT_EQ(dummy_participant->type()(), ParticipantKind::DUMMY);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Simple RTPS participant
 */
TEST(ParticipantFactoryTest, create_simple_participant)
{
    std::shared_ptr<IParticipant> simple_participant = test::create_participant("local", ParticipantKind::SIMPLE_RTPS);
    ASSERT_EQ(simple_participant->type()(), ParticipantKind::SIMPLE_RTPS);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Invalid participant, should throw \c ConfigurationException
 */
TEST(ParticipantFactoryTest, create_invalid_participant)
{
    ASSERT_THROW(test::create_participant("invalid_part", ParticipantKind::VOID), ConfigurationException);
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

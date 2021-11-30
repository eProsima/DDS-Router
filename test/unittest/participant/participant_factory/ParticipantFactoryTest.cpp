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
#include <ddsrouter/configuration/SimpleParticipantConfiguration.hpp>
#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/exceptions/ConfigurationException.hpp>
#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/participant/ParticipantFactory.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>
#include <ddsrouter/types/participant/ParticipantType.hpp>

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::rtps;

namespace eprosima {
namespace ddsrouter {
namespace test {

std::shared_ptr<IParticipant> create_participant(
        std::string id_str)
{
    ParticipantFactory participant_factory;
    std::shared_ptr<PayloadPool> payload_pool = std::make_shared<PayloadPool>();
    std::shared_ptr<DiscoveryDatabase> discovery_database = std::make_shared<DiscoveryDatabase>();
    ParticipantId id(id_str);
    ParticipantConfiguration* participant_configuration;
    if (id_str == "local")
    {
        ParticipantConfiguration parent_participant_configuration(id);
        participant_configuration = new SimpleParticipantConfiguration(parent_participant_configuration);
    }
    else
    {
        participant_configuration = new ParticipantConfiguration(id);
    }

    return participant_factory.create_participant(*participant_configuration, payload_pool, discovery_database);
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
    std::shared_ptr<IParticipant> void_participant = test::create_participant("void");
    ASSERT_EQ(void_participant->type()(), ParticipantType::VOID);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Echo participant
 */
TEST(ParticipantFactoryTest, create_echo_participant)
{
    std::shared_ptr<IParticipant> echo_participant = test::create_participant("echo");
    ASSERT_EQ(echo_participant->type()(), ParticipantType::ECHO);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Dummy participant
 */
TEST(ParticipantFactoryTest, create_dummy_participant)
{
    std::shared_ptr<IParticipant> dummy_participant = test::create_participant("dummy");
    ASSERT_EQ(dummy_participant->type()(), ParticipantType::DUMMY);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Simple RTPS participant
 */
TEST(ParticipantFactoryTest, create_simple_participant)
{
    std::shared_ptr<IParticipant> simple_participant = test::create_participant("local");
    ASSERT_EQ(simple_participant->type()(), ParticipantType::SIMPLE_RTPS);
}

/**
 * Test \c ParticipantFactory \c create_participant method
 *
 * CASES:
 *  Invalid participant, should throw \c ConfigurationException
 */
TEST(ParticipantFactoryTest, create_invalid_participant)
{
    ASSERT_THROW(test::create_participant("invalid_part"), ConfigurationException);
}

/**
 * Test \c ParticipantFactory \c remove_participant method
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

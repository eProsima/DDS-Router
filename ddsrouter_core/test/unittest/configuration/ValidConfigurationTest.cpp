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

#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

using namespace eprosima::ddsrouter;

/**
 * Test the correcteness or failure depending on the number of participants.
 * It uses blank and simple participants as it is simpler.
 *
 * POSITIVE CASES:
 * - 2 participants
 * - N participants
 * - 1 participant repeater
 *
 * NEGATIVE CASES:
 * - 0 participants
 * - 1 participant no repeater
 */
TEST(ValidConfigurationTest, number_of_participants)
{
    // 2 participants
    {
        std::set<std::shared_ptr<core::configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(
            test::random_participant_configuration(core::types::ParticipantKind::blank, 0));
        participant_configurations.insert(
            test::random_participant_configuration(core::types::ParticipantKind::blank, 1));

        core::configuration::DDSRouterConfiguration configuration(
                {},
                {},
                {},
                participant_configurations
            );

        utils::Formatter formatter;
        ASSERT_TRUE(configuration.is_valid(formatter));
    }

    // N participants
    {
        std::set<std::shared_ptr<core::configuration::ParticipantConfiguration>> participant_configurations;
        for (int i = 0; i < 10; ++i)
        {
            participant_configurations.insert(
                test::random_participant_configuration(core::types::ParticipantKind::blank, i));
        }

        core::configuration::DDSRouterConfiguration configuration(
                {},
                {},
                {},
                participant_configurations
            );

        utils::Formatter formatter;
        ASSERT_TRUE(configuration.is_valid(formatter));
    }

    // 1 participant repeater
    {
        std::set<std::shared_ptr<core::configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(
            std::make_shared<core::configuration::SimpleParticipantConfiguration>
                (
                    test::random_participant_id(),
                    core::types::ParticipantKind::simple_rtps,
                    test::random_domain(),
                    true
                )
            );

        core::configuration::DDSRouterConfiguration configuration(
                {},
                {},
                {},
                participant_configurations
            );

        utils::Formatter formatter;
        ASSERT_TRUE(configuration.is_valid(formatter)) << formatter.to_string();
    }

    // 0 participants
    {
        // Empty participant_configurations
        std::set<std::shared_ptr<core::configuration::ParticipantConfiguration>> participant_configurations;

        core::configuration::DDSRouterConfiguration configuration(
                {},
                {},
                {},
                participant_configurations
            );

        utils::Formatter formatter;
        ASSERT_FALSE(configuration.is_valid(formatter));
    }

    // 1 participant no repeater
    {
        std::set<std::shared_ptr<core::configuration::ParticipantConfiguration>> participant_configurations;
        participant_configurations.insert(
            test::random_participant_configuration(core::types::ParticipantKind::blank, 0));

        core::configuration::DDSRouterConfiguration configuration(
                {},
                {},
                {},
                participant_configurations
            );

        utils::Formatter formatter;
        ASSERT_FALSE(configuration.is_valid(formatter));
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

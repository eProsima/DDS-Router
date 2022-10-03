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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

// using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/*
 * Test \c ParticipantKind int conversions
 */
TEST(ParticipantKindTest, int_conversions)
{
    for (auto kind : ALL_PARTICIPANT_KINDS)
    {
        ASSERT_EQ(ALL_PARTICIPANT_KINDS[static_cast<ParticipantKindType>(kind)], kind);
    }
}

/*
 * Test \c ParticipantKind string conversions
 */
TEST(ParticipantKindTest, string_conversions)
{
    // Check consistency between enum value and its associated string.
    ASSERT_EQ(std::string(
                PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::invalid)]),
            std::string("invalid"));
    ASSERT_EQ(std::string(
                PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::blank)]), std::string(
                "blank"));
    ASSERT_EQ(std::string(PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::echo)]),
            std::string("echo"));
    ASSERT_EQ(std::string(
                PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::dummy)]), std::string(
                "dummy"));
    ASSERT_EQ(std::string(
                PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::simple_rtps)]),
            std::string("simple-rtps"));
    ASSERT_EQ(std::string(PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::
                    local_discovery_server)]), std::string("local-discovery-server"));
    ASSERT_EQ(std::string(
                PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::wan_initial_peers)]),
            std::string("wan-initial-peers"));
    ASSERT_EQ(std::string(
                PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::wan_discovery_server)]),
            std::string("wan-ds"));

    // Test all possible aliases for each participant kind

    // Strings mapping to ParticipantKind::invalid
    ASSERT_EQ(participant_kind_from_name(""), ParticipantKind::invalid);
    ASSERT_EQ(participant_kind_from_name("unexisting-kind"), ParticipantKind::invalid);
    ASSERT_EQ(participant_kind_from_name("__invalid_participant_kind__"), ParticipantKind::invalid);

    // Strings mapping to ParticipantKind::blank
    ASSERT_EQ(participant_kind_from_name("blank"), ParticipantKind::blank);
    ASSERT_EQ(participant_kind_from_name("void"), ParticipantKind::blank);

    // Strings mapping to ParticipantKind::echo
    ASSERT_EQ(participant_kind_from_name("echo"), ParticipantKind::echo);

    // Strings mapping to ParticipantKind::dummy
    ASSERT_EQ(participant_kind_from_name("dummy"), ParticipantKind::dummy);

    // Strings mapping to ParticipantKind::simple_rtps
    ASSERT_EQ(participant_kind_from_name("local"), ParticipantKind::simple_rtps);
    ASSERT_EQ(participant_kind_from_name("simple"), ParticipantKind::simple_rtps);

    // Strings mapping to ParticipantKind::local_discovery_server
    ASSERT_EQ(participant_kind_from_name("discovery-server"), ParticipantKind::local_discovery_server);
    ASSERT_EQ(participant_kind_from_name("ds"), ParticipantKind::local_discovery_server);
    ASSERT_EQ(participant_kind_from_name("local-ds"), ParticipantKind::local_discovery_server);
    ASSERT_EQ(participant_kind_from_name("local-discovery-server"), ParticipantKind::local_discovery_server);

    // Strings mapping to ParticipantKind::wan_initial_peers
    ASSERT_EQ(participant_kind_from_name("wan"), ParticipantKind::wan_initial_peers);
    ASSERT_EQ(participant_kind_from_name("router"), ParticipantKind::wan_initial_peers);

    // Strings mapping to ParticipantKind::wan_discovery_server
    ASSERT_EQ(participant_kind_from_name("wan-ds"), ParticipantKind::wan_discovery_server);
    ASSERT_EQ(participant_kind_from_name("wan-discovery-server"), ParticipantKind::wan_discovery_server);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

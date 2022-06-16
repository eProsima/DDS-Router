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

#include <array>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/*
 * List of valid names for participants
 */
constexpr std::array<const char*, 3> SomeValidNames =
{
    "BARRO_p",
    "lan_participant",
    "wan",
};

/*
 * List of non valid names for participants
 */
constexpr std::array<const char*, 2> AllNonValidNames =
{
    "",
    InvalidParticipantName,
};

/******************
* POSITIVE/NEGATIVE CASES *
******************/

/**
 * Test static \c ParticipantName \c is_valid_id method
 */
TEST(ParticipantIdTest, check_name_valid_invalid)
{
    for (auto id : SomeValidNames)
    {
        ASSERT_TRUE(is_participant_name_valid(id));
    }

    for (auto id : AllNonValidNames)
    {
        ASSERT_FALSE(is_participant_name_valid(id));
    }
}

/*
 * Test \c ParticipantKind arrays sizes
 */
TEST(ParticipantIdTest, kind_arrays_sizes)
{
    ASSERT_EQ(ALL_PARTICIPANT_KINDS.size(), PARTICIPANT_KIND_STRINGS.size());
    ASSERT_EQ(ALL_VALID_PARTICIPANT_KINDS.size() + 1, ALL_PARTICIPANT_KINDS.size());
}

/*
 * Test \c ParticipantKind int conversions
 */
TEST(ParticipantIdTest, kind_int_conversions)
{
    for (auto pk : ALL_PARTICIPANT_KINDS)
    {
        ASSERT_EQ(ALL_PARTICIPANT_KINDS[static_cast<ParticipantKindType>(pk)], pk);
    }
}

/*
 * Test \c ParticipantKind string conversions
 */
TEST(ParticipantIdTest, kind_string_conversions)
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
            std::string("simple_rtps"));
    ASSERT_EQ(std::string(PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::
                    local_discovery_server)]), std::string("local_discovery_server"));
    ASSERT_EQ(std::string(PARTICIPANT_KIND_STRINGS[static_cast<ParticipantKindType>(ParticipantKind::wan)]),
            std::string("wan"));

    // Test all possible aliases for each participant kind
    ASSERT_EQ(participant_kind_from_string(""), ParticipantKind::invalid);
    ASSERT_EQ(participant_kind_from_string("unexisting-kind"), ParticipantKind::invalid);
    ASSERT_EQ(participant_kind_from_string("__invalid_participant_kind__"), ParticipantKind::invalid);

    ASSERT_EQ(participant_kind_from_string("blank"), ParticipantKind::blank);
    ASSERT_EQ(participant_kind_from_string("void"), ParticipantKind::blank);
    ASSERT_EQ(participant_kind_from_string("echo"), ParticipantKind::echo);
    ASSERT_EQ(participant_kind_from_string("dummy"), ParticipantKind::dummy);

    ASSERT_EQ(participant_kind_from_string("local"), ParticipantKind::simple_rtps);
    ASSERT_EQ(participant_kind_from_string("simple"), ParticipantKind::simple_rtps);

    ASSERT_EQ(participant_kind_from_string("discovery-server"), ParticipantKind::local_discovery_server);
    ASSERT_EQ(participant_kind_from_string("ds"), ParticipantKind::local_discovery_server);
    ASSERT_EQ(participant_kind_from_string("local-ds"), ParticipantKind::local_discovery_server);
    ASSERT_EQ(participant_kind_from_string("local-discovery-server"), ParticipantKind::local_discovery_server);

    ASSERT_EQ(participant_kind_from_string("wan"), ParticipantKind::wan);
    ASSERT_EQ(participant_kind_from_string("router"), ParticipantKind::wan);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

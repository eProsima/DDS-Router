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

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/*
 * List of valid names for participants
 */
std::vector<std::string> random_valid_ids()
{
    return
        {
            "BARRO_p",
            "lan_participant",
            "wan",
        };
}

/*
 * List of non valid names for participants
 */
std::vector<std::string> random_non_valid_ids()
{
    return
        {
            "",
            "__invalid_ddsrouter_participant__",
        };
}

/****************
* CONSTRUCTORS *
****************/

/**
 * Test ParticipantId constructor without arguments
 */
TEST(ParticipantIdTest, default_constructor)
{
    ParticipantId pi;
    ASSERT_FALSE(pi.is_valid());
}

/**
 * Test ParticipantId constructor with arguments
 */
TEST(ParticipantIdTest, constructor)
{
    for (std::string id : random_valid_ids())
    {
        ParticipantId pi(id);
        ASSERT_TRUE(pi.is_valid());
    }
}

/******************
* STATIC METHODS *
******************/

/**
 * Test static \c ParticipantId \c is_valid_id method
 */
TEST(ParticipantIdTest, invalid)
{
    ParticipantId pi = ParticipantId::invalid();
    ASSERT_FALSE(pi.is_valid());
}

/******************
* POSITIVE CASES *
******************/

/**
 * Test static \c ParticipantId \c is_valid_id method
 */
TEST(ParticipantIdTest, is_valid_id)
{
    for (std::string id : random_valid_ids())
    {
        ASSERT_TRUE(ParticipantId::is_valid_id(id));
    }
}

/**
 * Test \c ParticipantId \c is_valid method
 *
 * CASES:
 *  by default constructor
 *  by \c invalid getter
 *  by actual invalid name
 */
TEST(ParticipantIdTest, is_valid)
{
    // By default constructor
    {
        ParticipantId pi;
        ASSERT_FALSE(pi.is_valid());
    }

    // By invalid getter
    {
        ParticipantId pi = ParticipantId::invalid();
        ASSERT_FALSE(pi.is_valid());
    }

    // By actual invalid name
    {
        ParticipantId pi("__invalid_ddsrouter_participant__");
        ASSERT_FALSE(pi.is_valid());
    }
}

/**
 * Test  \c ParticipantId \c == operator
 */
TEST(ParticipantIdTest, equal_operator)
{
    for (std::string id : random_valid_ids())
    {
        ParticipantId pi1(id);
        ParticipantId pi2(id);
        ASSERT_TRUE(pi1 == pi2);
    }
}

/**
 * Test  \c ParticipantId \c < operator
 */
TEST(ParticipantIdTest, minor_operator)
{
    std::vector<std::string> ids = random_valid_ids();

    // Sort ids by string comparison
    std::sort(ids.begin(), ids.end());

    for (unsigned int i = 0; i < ids.size(); ++i)
    {
        // Remove equal case
        for (unsigned int j = (i + 1); j < ids.size(); ++j)
        {
            ParticipantId pi1(ids[i]);
            ParticipantId pi2(ids[j]);
            ASSERT_TRUE(pi1 < pi2);
        }
    }
}

/******************
* NEGATIVE CASES *
******************/

/**
 * Test static \c ParticipantId \c is_valid_id method in negative cases
 */
TEST(ParticipantIdTest, is_non_valid_id)
{
    for (std::string id : random_non_valid_ids())
    {
        ASSERT_FALSE(ParticipantId::is_valid_id(id));
    }
}

/**
 * Test \c ParticipantId \c is_valid method in negative cases
 *
 * NOTE: this test is the same as the non default constructor: \c constructor
 */
TEST(ParticipantIdTest, is_non_valid)
{
    for (std::string id : random_valid_ids())
    {
        ParticipantId pi(id);
        ASSERT_TRUE(pi.is_valid());
    }
}

/**
 * Test  \c ParticipantId \c == operator in negative cases
 */
TEST(ParticipantIdTest, non_equal_operator)
{
    std::vector<std::string> ids = random_valid_ids();

    for (unsigned int i = 0; i < ids.size(); ++i)
    {
        for (unsigned int j = 0; j < ids.size(); ++j)
        {
            // Remove equal case
            if (i != j)
            {
                ParticipantId pi1(ids[i]);
                ParticipantId pi2(ids[j]);
                ASSERT_FALSE(pi1 == pi2);
            }
        }
    }
}

/**
 * Test  \c ParticipantId \c < operator in negative cases
 */
TEST(ParticipantIdTest, non_minor_operator)
{
    std::vector<std::string> ids = random_valid_ids();

    // Sort ids by string comparison
    std::sort(ids.begin(), ids.end());

    for (unsigned int i = 0; i < ids.size(); ++i)
    {
        // Remove equal case
        for (unsigned int j = i; j < ids.size(); ++j)
        {
            ParticipantId pi1(ids[i]);
            ParticipantId pi2(ids[j]);
            ASSERT_FALSE(pi2 < pi1);
        }
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

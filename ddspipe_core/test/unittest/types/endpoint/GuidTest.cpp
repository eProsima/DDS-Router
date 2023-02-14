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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <fastrtps/rtps/common/Guid.h>
#include <fastdds/rtps/common/EntityId_t.hpp>

#include <ddspipe_core/types/dds/Guid.hpp>

using namespace eprosima::ddspipe::core;
using namespace eprosima::ddspipe::core::types;

/**
 * Test \c Guid \c is_valid method
 *
 * CASES:
 *  Default constructor invalid
 *  Unknown guid invalid
 *  GuidPrefix invalid
 *  EntityId invalid
 *  Constrcutor with valid value
 */
TEST(GuidTest, is_valid)
{
    // Default constructor invalid
    {
        Guid invalid_guid;
        ASSERT_FALSE(invalid_guid.is_valid());
    }

    // Unknown guid invalid
    {
        Guid invalid_guid_1(
            eprosima::fastrtps::rtps::GuidPrefix_t::unknown(),
            eprosima::fastrtps::rtps::EntityId_t::unknown());
        ASSERT_FALSE(invalid_guid_1.is_valid());

        Guid invalid_guid_2(
            eprosima::fastrtps::rtps::GUID_t::unknown().guidPrefix,
            eprosima::fastrtps::rtps::GUID_t::unknown().entityId);
        ASSERT_FALSE(invalid_guid_2.is_valid());
    }

    // GuidPrefix invalid
    {
        Guid invalid_guid(
            eprosima::fastrtps::rtps::GuidPrefix_t(), // Invalid GuidPrefix
            eprosima::fastrtps::rtps::EntityId_t(1)); // Valid EntityId
        ASSERT_FALSE(invalid_guid.is_valid());
    }

    // EntityId invalid
    {
        eprosima::fastrtps::rtps::GuidPrefix_t guid_prefix;
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> guid_prefix;
        Guid valid_guid(
            guid_prefix,                             // Valid GuidPrefix
            eprosima::fastrtps::rtps::EntityId_t()); // Invalid EntityId
        ASSERT_FALSE(valid_guid.is_valid());
    }

    // Constrcutor with valid value
    {
        eprosima::fastrtps::rtps::GuidPrefix_t guid_prefix;
        std::istringstream("44.53.00.5f.45.50.52.4f.53.49.4d.41") >> guid_prefix;
        Guid valid_guid(
            guid_prefix,                              // Valid GuidPrefix
            eprosima::fastrtps::rtps::EntityId_t(1)); // Valid EntityId
        ASSERT_TRUE(valid_guid.is_valid());
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

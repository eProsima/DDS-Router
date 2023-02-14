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

#include <ddspipe_core/testing/random_values.hpp>
#include <ddspipe_core/types/dds/Endpoint.hpp>

using namespace eprosima::ddspipe::core;
using namespace eprosima::ddspipe::core::types;
using namespace eprosima::ddspipe::core::testing;

namespace test
{

constexpr const unsigned int TEST_ITERATIONS = 100;

} // test


/**
 * Test \c Endpoint \c topic getter method
 *
 * CASES:
 *  Random Topics
 */
TEST(EndpointTest, topic_qos_getter)
{
    for (unsigned int i = 0; i < test::TEST_ITERATIONS; i++)
    {
        Endpoint endpoint = random_endpoint(i);
        ASSERT_EQ(endpoint.topic.topic_qos, endpoint.topic_qos());
    }
}

/**
 * Test \c Endpoint \c is_writer getter method
 *
 * CASES:
 *  Writer
 *  Reader
 */
TEST(EndpointTest, is_reader_writer)
{
    // Default
    {
        Endpoint endpoint;
        ASSERT_FALSE(endpoint.is_writer());
        ASSERT_FALSE(endpoint.is_reader());
    }

    // Writer
    {
        Endpoint endpoint;
        endpoint.kind = EndpointKind::writer;
        ASSERT_TRUE(endpoint.is_writer());
        ASSERT_FALSE(endpoint.is_reader());
    }

    // Reader
    {
        Endpoint endpoint;
        endpoint.kind = EndpointKind::reader;
        ASSERT_FALSE(endpoint.is_writer());
        ASSERT_TRUE(endpoint.is_reader());
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

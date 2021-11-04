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

#include <databroker/types/endpoint/QoS.hpp>

using namespace eprosima::databroker;

/**
 * Test \c QoS constructor using Fast DDS QoS values
 */
TEST(QoSTest, constructor)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c QoS \c durability getter method
 */
TEST(QoSTest, durability_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

/**
 * Test \c QoS \c reliability getter method
 */
TEST(QoSTest, reliability_getter)
{
    // TODO
    ASSERT_TRUE(false);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

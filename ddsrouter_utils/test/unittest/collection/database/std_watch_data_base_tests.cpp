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

#include <ddsrouter_utils/wait/IntWaitHandler.hpp>
#include <ddsrouter_utils/collection/database/StdWatchDataBase.hpp>
#include <ddsrouter_utils/thread/manager/StdThreadPool.hpp>

using namespace eprosima::ddsrouter::utils;

namespace test {

constexpr const unsigned int N_THREADS_IN_TEST = 3;
constexpr const unsigned int N_EXECUTIONS_IN_TEST = 20;

std::shared_ptr<thread::IManager> create_thread_manager()
{
    return std::make_shared<thread::StdThreadPool>(N_THREADS_IN_TEST, true);
}

} /* namespace test */

TEST(StdWatchDataBaseTest, basic_functionality_test)
{
    // Create Database
    StdWatchDataBase<int, std::string> database(test::create_thread_manager());

    // Add value
    database.add(1, "number 1");

    // Check that value exist
    ASSERT_TRUE(database.exist(1));

    // Get value
    ASSERT_EQ(database.get(1), "number 1");

    // Modify value
    database.modify(1, "number 1 modified");

    // Get value
    ASSERT_EQ(database.get(1), "number 1 modified");

    // Remove value
    database.remove(1);

    // Check that value does not exist anymore
    ASSERT_FALSE(database.exist(1));
}

TEST(StdWatchDataBaseTest, sync_callback_functionality_test)
{
    // Use "global" variables to know the arguments to the callback
    std::pair<int, std::string> addition_callback_result;
    std::pair<int, std::string> modification_callback_result;
    std::pair<int, std::string> remove_callback_result;

    // Create Database
    StdWatchDataBase<int, std::string> database(test::create_thread_manager());

    // Add addition callback
    database.register_addition_callback(
        [&addition_callback_result]
        (int key, std::string value)
        {
            addition_callback_result.first = key;
            addition_callback_result.second = value;
        }
    );

    // Add value
    database.sync_add(1, "number 1");

    // Wait for counter to be raised once and check the arguments
    ASSERT_EQ(addition_callback_result.first, 1);
    ASSERT_EQ(addition_callback_result.second, "number 1");

    // Add modification callback
    database.register_modification_callback(
        [&modification_callback_result]
        (int key, std::string value)
        {
            modification_callback_result.first = key;
            modification_callback_result.second = value;
        }
    );

    // Add value
    database.sync_modify(1, "number 1 modified");

    // Wait for counter to be raised once and check the arguments
    ASSERT_EQ(modification_callback_result.first, 1);
    ASSERT_EQ(modification_callback_result.second, "number 1 modified");

    // Add remove callback
    database.register_deletion_callback(
        [&remove_callback_result]
        (int key, std::string value)
        {
            remove_callback_result.first = key;
            remove_callback_result.second = value;
        }
    );

    // Add value
    database.sync_remove(1);

    // Wait for counter to be raised once and check the arguments
    ASSERT_EQ(remove_callback_result.first, 1);
    ASSERT_EQ(remove_callback_result.second, "number 1 modified");
}

TEST(StdWatchDataBaseTest, semisync_callback_functionality_test)
{
    // Use "global" variables and a counter to know when the callback has been called and its arguments
    eprosima::ddsrouter::event::IntWaitHandler counter(0);
    std::pair<int, std::string> addition_callback_result;
    std::pair<int, std::string> modification_callback_result;
    std::pair<int, std::string> remove_callback_result;

    // Create Database
    StdWatchDataBase<int, std::string> database(test::create_thread_manager());

    // Add addition callback
    database.register_addition_callback(
        [&addition_callback_result, &counter]
        (int key, std::string value)
        {
            addition_callback_result.first = key;
            addition_callback_result.second = value;
            ++counter;
        }
    );

    // Add value
    database.add(1, "number 1");

    // Wait for counter to be raised once and check the arguments
    counter.wait_equal(1);
    ASSERT_EQ(addition_callback_result.first, 1);
    ASSERT_EQ(addition_callback_result.second, "number 1");
    counter.set_value(0);

    // Add modification callback
    database.register_modification_callback(
        [&modification_callback_result, &counter]
        (int key, std::string value)
        {
            modification_callback_result.first = key;
            modification_callback_result.second = value;
            ++counter;
        }
    );

    // Add value
    database.modify(1, "number 1 modified");

    // Wait for counter to be raised once and check the arguments
    counter.wait_equal(1);
    ASSERT_EQ(modification_callback_result.first, 1);
    ASSERT_EQ(modification_callback_result.second, "number 1 modified");
    counter.set_value(0);

    // Add remove callback
    database.register_deletion_callback(
        [&remove_callback_result, &counter]
        (int key, std::string value)
        {
            remove_callback_result.first = key;
            remove_callback_result.second = value;
            ++counter;
        }
    );

    // Add value
    database.remove(1);

    // Wait for counter to be raised once and check the arguments
    counter.wait_equal(1);
    ASSERT_EQ(remove_callback_result.first, 1);
    ASSERT_EQ(remove_callback_result.second, "number 1 modified");
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

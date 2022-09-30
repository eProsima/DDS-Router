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
#include <ddsrouter_utils/types/Atomicable.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/math/math.hpp>
#include <ddsrouter_utils/thread/manager/StdThreadPool.hpp>
#include <ddsrouter_utils/thread/connector/SlotConnector.hpp>

using namespace eprosima::ddsrouter;

namespace test {

utils::Duration_ms DEFAULT_TIME_TEST = 20u;
unsigned int DEFAULT_TIME_REPETITIONS = 20u;
unsigned int DEFAULT_THREADS = 3u;

/**
 * @brief Function that increases \c counter in \c increase so can be checked that has been successfully done.
 *
 * @param counter Wait Handler that holds the number of increases done to the same variable
 * @param increase value to increase \c counter
 */
void test_lambda_increase_waiter(
        event::IntWaitHandler& counter,
        int increase = 1)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIME_TEST));
    for (int i = 0; i < increase; ++i)
    {
        ++counter;
    }
}

void test_lambda_increase_waiter_add_string(
        event::IntWaitHandler& counter,
        utils::Atomicable<std::string>& bucket,
        std::string string_to_add,
        int increase = 1,
        bool append_string = true)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIME_TEST));

    // Lock string that will be modified
    if (append_string)
    {
        std::unique_lock<utils::Atomicable<std::string>> lock(bucket);
        bucket.append(string_to_add);
    }

    for (int i = 0; i < increase; ++i)
    {
        ++counter;
    }
}

/**
 * Manager type to use.
 * Using \c StdThreadPool because it is the one that will be used the most.
 */
using ManagerType = utils::thread::StdThreadPool;

utils::thread::IManager* create_manager()
{
    return new ManagerType(DEFAULT_THREADS, true);
}

} /* namespace eprosima */

/**
 * Construct a StdThreadPool and uses SlotConnector to send executions without parameters
 *
 * STEPS:
 * - Create Manager
 * - Call a SlotConnector by copying an already existing std::function
 * - Call SlotConnector N times with a new created lambda each time
 * - Check that the final value is the expected
 */
TEST(SlotConnectorTest, slot_test_no_params)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);

    // Manager object
    utils::thread::IManager* manager = test::create_manager();

    // Create lambda increasing in 1
    std::function<void()> lambda = [&counter](){ test::test_lambda_increase_waiter(counter, 1); };
    // Create slot by copy
    utils::thread::SimpleSlotConnector once_slot(manager, lambda);

    // Execute slot
    once_slot.execute();

    // Wait for lambda to be called required times
    counter.wait_equal(1);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), 1);
    // Reset counter
    counter.set_value(0);

    // Execute lambda N times by moving increasing in 1 by moving
    utils::thread::SimpleSlotConnector move_slot(manager, std::move(lambda));
    for (unsigned int i = 1; i <= test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        move_slot.execute();
    }

    // Wait for lambda to be called required times
    uint32_t target_value = test::DEFAULT_TIME_REPETITIONS;
    counter.wait_equal(target_value);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), target_value);

    // Erase Manager
    delete manager;
}

TEST(SlotConnectorTest, slot_test_int)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);

    // Manager object
    utils::thread::IManager* manager = test::create_manager();

    // Create lambda increasing in 1
    std::function<void(int)> lambda = [&counter](int x){ test::test_lambda_increase_waiter(counter, x); };
    // Create slot by copy
    utils::thread::SlotConnector<int> once_slot(manager, lambda);

    // Execute slot
    once_slot.execute(1);

    // Wait for lambda to be called required times
    counter.wait_equal(1);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), 1);
    // Reset counter
    counter.set_value(0);

    // Execute lambda N times by moving increasing in 1 by moving
    utils::thread::SlotConnector<int> move_slot(manager, std::move(lambda));
    for (unsigned int i = 1; i <= test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        move_slot.execute(i);
    }

    // Wait for lambda to be called required times
    uint32_t target_value = utils::arithmetic_progression_sum(1, 1, test::DEFAULT_TIME_REPETITIONS);
    counter.wait_equal(target_value);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), target_value);

    // Erase Manager
    delete manager;
}

TEST(SlotConnectorTest, slot_test_string)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);
    // String to check result
    utils::Atomicable<std::string> bucket;

    // Manager object
    utils::thread::IManager* manager = test::create_manager();

    // Create lambda increasing in 1
    std::function<void(std::string)> lambda =
        [&counter, &bucket](std::string st){ test::test_lambda_increase_waiter_add_string(counter, bucket, st, 1); };
    // Create slot by copy
    utils::thread::SlotConnector<std::string> once_slot(manager, lambda);

    // Execute slot
    once_slot.execute("Hello");

    // Wait for lambda to be called required times
    counter.wait_equal(1);
    // Check that lambda has been called only that amount of times and the string result is the correct
    ASSERT_EQ(counter.get_value(), 1);
    ASSERT_EQ(bucket, "Hello");  // It does not require mutex as the modification in test has already been done
    // Reset counter
    counter.set_value(0);
    bucket.erase();

    // Execute lambda N times by moving increasing in 1 by moving
    utils::thread::SlotConnector<std::string> move_slot(manager, std::move(lambda));
    for (unsigned int i = 1; i <= test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        move_slot.execute(std::string(1, static_cast<char>('a' + i - 1)));
    }

    // Wait for lambda to be called required times
    uint32_t target_value = test::DEFAULT_TIME_REPETITIONS;
    counter.wait_equal(target_value);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), target_value);

    // Check the result string. It may not be in the order expected as the order of threads is not deterministic
    // Thus, check that every char from 'a' to 'a' + N is in the string
    for (char c = 'a'; c < 'a' + test::DEFAULT_TIME_REPETITIONS; ++c)
    {
        ASSERT_NE(bucket.find(c), std::string::npos) << c;
    }

    // Erase Manager
    delete manager;
}

TEST(SlotConnectorTest, slot_test_bool_int_string)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);
    // String to check result
    utils::Atomicable<std::string> bucket;

    // Manager object
    utils::thread::IManager* manager = test::create_manager();

    // Create lambda increasing in n
    std::function<void(bool, int, std::string)> lambda =
        [&counter, &bucket]
        (bool b, int i, std::string s)
        { test::test_lambda_increase_waiter_add_string(counter, bucket, s, i, b); };

    // Create slot by copy
    utils::thread::SlotConnector<bool, int, std::string> once_slot(manager, lambda);

    // Execute slot
    once_slot.execute(true, 1, "Hello");

    // Wait for lambda to be called required times
    counter.wait_equal(1);
    // Check that lambda has been called only that amount of times and the string result is the correct
    ASSERT_EQ(counter.get_value(), 1);
    ASSERT_EQ(bucket, "Hello");  // It does not require mutex as the modification in test has already been done
    // Reset counter
    counter.set_value(0);
    bucket.erase();

    // Execute lambda N times by moving increasing in n by moving
    utils::thread::SlotConnector<bool, int, std::string> move_slot(manager, std::move(lambda));
    for (unsigned int i = 1; i <= test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        // Whether it should add the char. Only add odd number chars
        char c = static_cast<char>('a' + i - 1);
        bool append_char = static_cast<bool>(static_cast<int>(c) % 2);

        move_slot.execute(
            append_char,
            i,
            std::string(1, c));
    }

    // Wait for lambda to be called required times
    uint32_t target_value = utils::arithmetic_progression_sum(1, 1, test::DEFAULT_TIME_REPETITIONS);
    counter.wait_equal(target_value);
    // Check that lambda has been called only that amount of times
    ASSERT_EQ(counter.get_value(), target_value);

    // Check the result string. It may not be in the order expected as the order of threads is not deterministic
    // Thus, check that every char from 'a' to 'a' + N is in the string
    for (char c = 'a'; c < 'a' + test::DEFAULT_TIME_REPETITIONS; ++c)
    {
        bool append_char = static_cast<bool>(static_cast<int>(c) % 2);
        if (append_char)
        {
            ASSERT_NE(bucket.find(c), std::string::npos);
        }
        else
        {
            ASSERT_EQ(bucket.find(c), std::string::npos);
        }
    }

    // Erase Manager
    delete manager;
}

TEST(SlotConnectorTest, slot_test_complex_args)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);

    // Manager object
    utils::thread::IManager* manager = test::create_manager();

    // Use a function reference and already created values to call in the Pool
    utils::thread::SlotConnector<event::IntWaitHandler&, const int> move_slot(
        manager,
        test::test_lambda_increase_waiter);

    for (unsigned int i = 1; i <= test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        move_slot.execute(counter, static_cast<int>(i));
    }

    // Wait for lambda to be called required times
    uint32_t target_value = utils::arithmetic_progression_sum(1, 1, test::DEFAULT_TIME_REPETITIONS);
    counter.wait_equal(target_value);

    // Check that lambda has been called only that amount of times and the string result is the correct
    ASSERT_EQ(counter.get_value(), target_value);

    delete manager;
}

int main(
        int argc,
        char** argv)
{
    // utils::Log::SetVerbosity(utils::Log::Kind::Info);
    // utils::Log::Flush();

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

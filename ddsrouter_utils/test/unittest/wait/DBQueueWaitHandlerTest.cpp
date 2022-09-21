// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <ddsrouter_utils/wait/DBQueueWaitHandler.hpp>
#include <ddsrouter_utils/exception/DisabledException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {
namespace test {

eprosima::ddsrouter::utils::Duration_ms DEFAULT_TIME_TEST = 1000u;
eprosima::ddsrouter::utils::Duration_ms RESIDUAL_TIME_TEST = 10u;
eprosima::ddsrouter::utils::Duration_ms LONG_TIME_TEST = 5000u;

} /* namespace test */
} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::event;

/**
 * Check that pushing and popping values works as expected from the same thread
 * Using int as object to use inside collection.
 *
 * CASES:
 * - Push and pop one value
 * - Push and pop multiple values
 */
TEST(DBQueueWaitHandlerTest, push_pop_one_thread_int)
{
    // Push and pop one value
    {
        DBQueueWaitHandler<int> handler;
        handler.produce(1);
        EXPECT_EQ(handler.consume(), 1);
    }

    // Push and pop multiple values
    {
        DBQueueWaitHandler<int> handler;

        handler.produce(1);
        handler.produce(2);
        handler.produce(3);

        EXPECT_EQ(handler.consume(), 1);
        EXPECT_EQ(handler.consume(), 2);

        handler.produce(4);

        EXPECT_EQ(handler.consume(), 3);
        EXPECT_EQ(handler.consume(), 4);
    }
}

/**
 * Check that pushing and popping values works as expected from the same thread
 * Using std::string as object to use inside collection.
 *
 * CASES:
 * - Push and pop one value by moving it
 *
 * TODO:
 * This test will not work as DBQueue do not allow moving values, it only copies them
 */
TEST(DBQueueWaitHandlerTest, push_pop_one_thread_string_move)
{
    DBQueueWaitHandler<std::string> handler;

    std::string source_value("test_data");
    std::string lvalue(source_value);

    // This lvalue is moved as rvalue, so after moving it will be empty
    handler.produce(std::move(lvalue));
    // TODO uncomment it once DBQueue supports moving values
    ASSERT_EQ(lvalue.size(), 0);

    // Getting first value
    std::string pop_value = handler.consume();
    EXPECT_EQ(source_value, pop_value);
}

/**
 * Check that pushing and popping values works as expected from the same thread
 * Using std::string as object to use inside collection.
 *
 * CASES:
 * - Push and pop one value by reference
 */
TEST(DBQueueWaitHandlerTest, push_pop_one_thread_string_copy)
{
    DBQueueWaitHandler<std::string> handler;

    std::string lvalue("test_data");

    handler.produce(std::string(lvalue));

    // Getting first value
    std::string pop_value = handler.consume();
    EXPECT_EQ(lvalue, pop_value);

    // They should be different objects, check that modifying one does not modify the other
    pop_value[0] = 'a';
    EXPECT_NE(lvalue, pop_value);
}

/**
 * STEPS:
 * - 1
 * - Create 3 threads. A & B will read wait handler, and C will write
 * - Add 2 values [1, 2] to the handler (from C)
 * - Get 1 value from A and 1 from B
 * - 2
 * - Create 3 threads. A & B will read wait handler, and C will write
 * - Add 3 values [3, 4, 5] to the handler (from C)
 * - Get 1 value from A and 1 from B
 * - Get 1 value from A and 1 from B
 * - Disable handler
 * - Check A or B has stopped due to exception
 */
TEST(DBQueueWaitHandlerTest, push_one_thread_pop_many_int)
{
    // 1
    {
        bool popped_1 = false;
        bool popped_2 = false;
        DBQueueWaitHandler<int> handler;

        // Create 3 threads. A & B will read wait handler, and C will write
        std::thread thread_A([&]()
                {
                    int pop_value = handler.consume();
                    EXPECT_TRUE((pop_value == 1) || (pop_value == 2));
                    if (pop_value == 1)
                    {
                        popped_1 = true;
                    }
                    else
                    {
                        popped_2 = true;
                    }
                });
        std::thread thread_B([&]()
                {
                    int pop_value = handler.consume();
                    EXPECT_TRUE((pop_value == 1) || (pop_value == 2));
                    if (pop_value == 1)
                    {
                        popped_1 = true;
                    }
                    else
                    {
                        popped_2 = true;
                    }
                });
        std::thread thread_C([&]()
                {

                    handler.produce(1);
                    handler.produce(2);
                });

        thread_A.join();
        thread_B.join();
        thread_C.join();

        ASSERT_TRUE(popped_1 && popped_2);
    }

    // 2
    {
        int popped_1 = 0;
        int popped_2 = 0;
        int popped_3 = 0;
        int stopped_by_exception = 0;
        DBQueueWaitHandler<int> handler;

        auto lambda_for_A_and_B =
                [&popped_1, &popped_2, &popped_3, &stopped_by_exception, &handler]
                    ()
                {
                    try
                    {
                        // Wait for first value from queue
                        int pop_value = handler.consume();
                        if (pop_value == 1)
                        {
                            popped_1++;
                        }
                        else if (pop_value == 2)
                        {
                            popped_2++;
                        }
                        else if (pop_value == 3)
                        {
                            popped_3++;
                        }
                        // Wait for second value from queue
                        pop_value = handler.consume();
                        if (pop_value == 1)
                        {
                            popped_1++;
                        }
                        else if (pop_value == 2)
                        {
                            popped_2++;
                        }
                        else if (pop_value == 3)
                        {
                            popped_3++;
                        }
                    }
                    catch (const eprosima::ddsrouter::utils::DisabledException& e)
                    {
                        // Stopped by disabled
                        stopped_by_exception++;
                    }

                };

        // Create 3 threads. A & B will read wait handler, and C will write
        std::thread thread_A(lambda_for_A_and_B);
        std::thread thread_B(lambda_for_A_and_B);

        std::thread thread_C([&]()
                {
                    handler.produce(1);
                    handler.produce(2);
                    handler.produce(3);
                });

        thread_C.join();

        // Wait for notification to arrive to waiting threads
        std::this_thread::sleep_for(std::chrono::milliseconds(test::RESIDUAL_TIME_TEST));

        handler.disable();

        thread_A.join();
        thread_B.join();

        ASSERT_TRUE(1 == popped_1);
        ASSERT_TRUE(1 == popped_2);
        ASSERT_TRUE(1 == popped_3);
        ASSERT_TRUE(1 == stopped_by_exception);
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

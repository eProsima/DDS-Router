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
#include <ddsrouter_utils/time/Timer.hpp>
#include <ddsrouter_utils/time/time_utils.hpp>

#include <ddsrouter_utils/thread/task/OwnedTask.hpp>
#include <ddsrouter_utils/thread/task/ReferenceTask.hpp>
#include <ddsrouter_utils/thread/task/ArgsOwnedTask.hpp>

using namespace eprosima::ddsrouter;

namespace test {

utils::Duration_ms DEFAULT_TIME_TEST = 20u;
unsigned int DEFAULT_TIME_REPETITIONS = 20u;

/**
 * @brief Function that increases \c counter in \c increase so can be checked that has been successfully done.
 *
 * @param counter Wait Handler that holds the number of increases done to the same variable
 * @param increase value to increase \c counter
 */
void test_lambda_increase_waiter(
        event::IntWaitHandler& counter,
        unsigned int increase = 1)
{
    utils::sleep_for(DEFAULT_TIME_TEST);
    for (unsigned int i = 0; i < increase; ++i)
    {
        ++counter;
    }
}

template <class Task>
utils::thread::ITask* create_task_specialization(std::function<void()>* callback);

template <>
utils::thread::ITask* create_task_specialization<utils::thread::ReferenceTask>(
    std::function<void()>* callback)
{
    // Copy callback value inside new object
    return new utils::thread::ReferenceTask(callback);
}

template <>
utils::thread::ITask* create_task_specialization<utils::thread::OwnedTask>(
    std::function<void()>* callback)
{
    // Copy callback value inside new object
    return new utils::thread::OwnedTask(*callback);
}

template <>
utils::thread::ITask* create_task_specialization<utils::thread::ArgsOwnedTask<>>(
    std::function<void()>* callback)
{
    // Copy callback value inside new object
    return new utils::thread::ArgsOwnedTask<>(*callback);
}

template <>
utils::thread::ITask* create_task_specialization<utils::thread::ArgsOwnedTask<int>>(
    std::function<void()>* callback)
{
    // Copy callback value inside new object
    return new utils::thread::ArgsOwnedTask<int>(
        [callback](int x){ callback->operator()(); },
        1);
}

} /* namespace test */

// Empty class to parametrized tests
template<class T>
struct ThreadTaskTest : public ::testing::Test
{};
// Needed gtest macro
TYPED_TEST_SUITE_P(ThreadTaskTest);

/**
 * This tests operator() of every specialization if ITask
 *
 * Uses a IntWaitHandler to increase a value and at the same time wait for it to be updated to a exact value.
 * It is increased from a function executed inside the ITask, and the wait for it to be updated and check the value.
 *
 */
TYPED_TEST_P(ThreadTaskTest, task_operator)
{
    // Waiter to check result
    event::IntWaitHandler counter(0);

    // Function object to create tasks
    std::function<void()> task_function(
        [&counter](){ test::test_lambda_increase_waiter(counter, 1); });

    // Create task
    utils::thread::ITask* task(
        test::create_task_specialization<TypeParam>(
            &task_function));

    // Execute lambda 1 time
    task->operator()();
    counter.wait_equal(1);
    counter.set_value(0);

    // Execute lambda N times
    for (unsigned int i = 0; i < test::DEFAULT_TIME_REPETITIONS; ++i)
    {
        task->operator()();
    }
    counter.wait_equal(test::DEFAULT_TIME_REPETITIONS);

    // Erase Task
    delete task;
}

// Register test class and test cases
REGISTER_TYPED_TEST_SUITE_P(
    ThreadTaskTest,
    task_operator
);

// Set types used in parametrization
typedef ::testing::Types<
        utils::thread::ReferenceTask,
        utils::thread::OwnedTask,
        utils::thread::ArgsOwnedTask<>,
        utils::thread::ArgsOwnedTask<int>
    > CaseTypes;

// Generate each test case for each type case
INSTANTIATE_TYPED_TEST_SUITE_P(
    ParametrizedThreadTaskTest,
    ThreadTaskTest,
    CaseTypes);

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

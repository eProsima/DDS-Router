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

#include <atomic>
#include <functional>
#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <TestLogHandler.hpp>

#include <fastdds/rtps/common/SerializedPayload.h>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/time/Timer.hpp>
#include <ddsrouter_event/wait/BooleanWaitHandler.hpp>
#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/types/dds/Data.hpp>
#include <efficiency/CopyPayloadPool.hpp>
#include <efficiency/MapPayloadPool.hpp>
#include <efficiency/FastPayloadPool.hpp>
#include <participant/implementations/auxiliar/MockParticipant.hpp>

namespace eprosima {
namespace fastrtps {
namespace rtps {

/*
 * WORKAROUND:
 * This definition is needed due to googletest-distribution (1.11.0) requires to every class used inside ASSERT macro
 * to have the operator << defined in SAME namespace than the class.
 * In our case, Payload is defined as eprosima::fastrtps::rtps::SerializedPayload_t but redefined as
 * eprosima::ddsrouter::core::types::Payload and the operator << is defined in eprosima::ddsrouter::core::types
 * Thus, gtest could not find this definition (arising a very messy and cryptic compilation error).
 * This definition corrects that problem.
 *
 * NOTE:
 * In googletest-distribution release-1.10.0 this does not happen.
 */
void PrintTo(
        const SerializedPayload_t,
        std::ostream* os)
{
    *os << "::eprosima::fastrtps::rtps::SerializedPayload_t";
}

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

namespace test {

/**
 * @brief Execute a function N times for each combination of arguments, calculate the time elapsed and print it
 *
 * @param output_stream
 * @param column_names
 * @param test_repetitions
 * @param arguments
 * @param test_function
 * @param separator
 *
 * @TODO this method should be in test utils or somwhere general to reuse it
 */
void execute_performance_test(
    std::ostream& output_stream,
    const std::vector<std::string>& column_names,
    const uint32_t test_repetitions,
    const std::vector<std::vector<uint32_t>>& arguments,
    const std::function<double (const std::vector<uint32_t>&)>& test_function,
    const std::string& separator = ";")
{
    // TODO check column names and arguments are the same

    // TODO Write column names (add time column)

    // Create index vector to iterate over arguments
    std::vector<uint32_t> index_vector(arguments.size(), 0);
    bool exit = false;

    // Iterate over all arguments
    while (!exit)
    {
        // Get vector of arguments to test
        {
            std::vector<uint32_t> arguments_to_test;
            for (uint32_t argument_index=0 ; argument_index < arguments.size() ; ++argument_index)
            {
                arguments_to_test.push_back(arguments[argument_index][index_vector[argument_index]]);
            }

            double time_sum = 0;
            for (uint32_t test_index=0 ; test_index < test_repetitions ; ++test_index)
            {
                // Execute test
                time_sum += test_function(arguments_to_test);
            }

            // Store time in ostream
            eprosima::ddsrouter::utils::element_container_to_stream<uint32_t>(output_stream, arguments_to_test, separator);
            output_stream << separator << time_sum << std::endl;
        }

        // Increase index vector (done by copilot, so cool)
        for (uint32_t i = 0; i < index_vector.size(); ++i)
        {
            ++index_vector[i];
            if (index_vector[i] < arguments[i].size())
            {
                break;
            }
            else
            {
                index_vector[i] = 0;

                if (i == index_vector.size() - 1)
                {
                    exit = true;
                }
            }
        }
    }
}

void payload_pool_performance_test_routine_thread(
    eprosima::ddsrouter::event::BooleanWaitHandler* wait_handler,
    eprosima::ddsrouter::core::PayloadPool* payload_pool,
    uint32_t payload_size,
    uint32_t number_of_allocations)
{
    eprosima::fastrtps::rtps::IPayloadPool* this_pool = payload_pool;

    // Wait for test to start
    wait_handler->wait();

    // Allocate first payload
    eprosima::ddsrouter::core::types::Payload payload_src;
    payload_pool->get_payload(payload_size, payload_src);

    // Allocate rest of payloads
    for (uint32_t allocation=1 ; allocation < number_of_allocations ; ++allocation)
    {
        eprosima::ddsrouter::core::types::Payload payload_target;
        // Get payload
        payload_pool->get_payload(payload_src, this_pool, payload_target);

        payload_pool->release_payload(payload_target);
    }

    payload_pool->release_payload(payload_src);
}

/**
 * @brief
 *
 * @param arguments
 *  Data Size to allocate
 *  Number of allocations
 *  Number of threads
 *
 * @return time elapsed
 */
template <class PP>
double payload_pool_performance_test(
        std::vector<uint32_t> arguments)
{
    // TODO check there are 3 arguments

    // Get arguments
    uint32_t data_size = arguments[0];
    uint32_t number_of_allocations = arguments[1];
    uint32_t number_of_threads = arguments[2];

    // Create Wait Handler to threads do not start before everything is set up
    eprosima::ddsrouter::event::BooleanWaitHandler wait_handler(false);

    // Create payload pool
    PP payload_pool;

    // Create threads
    std::vector<std::thread> threads;
    for (uint32_t thread_index=0 ; thread_index < number_of_threads ; ++thread_index)
    {
        threads.push_back(
            std::thread(
                payload_pool_performance_test_routine_thread,
                &wait_handler,
                &payload_pool,
                data_size,
                number_of_allocations));
    }

    // Start Timer
    eprosima::ddsrouter::utils::Timer timer;

    // Open wait handler and let threads work
    wait_handler.open();

    // Get Time
    double time_elapsed = timer.elapsed();

    // Then join all threads and finish
    for (uint32_t thread_index=0 ; thread_index < number_of_threads ; ++thread_index)
    {
        threads[thread_index].join();
    }

    // Deallocate every payload
    // Will be removed when payload pool is destroyed

    return time_elapsed;
}

constexpr const uint32_t REPETITION_TEST = 100;

template <class PP>
void payload_pool_performance_templatized_test()
{
    // Prepare column names
    std::vector<std::string> column_names{
        "Data Size",
        "Number of Allocations",
        "Number of Threads"
    };

    // Prepare argument vectors
    // std::vector<uint32_t> data_sizes{8, 32, 256, 2048, 1048576};
    // std::vector<uint32_t> number_of_threads{1, 2, 4, 8, 16, 32};
    // std::vector<uint32_t> number_of_allocations{1, 2, 16, 32, 1024};
    std::vector<uint32_t> data_sizes{8, 2048, 1048576};
    std::vector<uint32_t> number_of_allocations{1, 2, 32};
    std::vector<uint32_t> number_of_threads{1, 4, 32};

    // Launch test and print results in stdout
    ::test::execute_performance_test(
        std::cout,
        column_names,
        ::test::REPETITION_TEST,
        std::vector<std::vector<uint32_t>>{data_sizes, number_of_allocations, number_of_threads},
        ::test::payload_pool_performance_test<PP>);
}

} /* namespace test */

using namespace eprosima::ddsrouter;

TEST(PayloadPoolPerformanceTest, copy_payload_pool)
{
    ::test::payload_pool_performance_templatized_test<core::CopyPayloadPool>();
}

TEST(PayloadPoolPerformanceTest, map_payload_pool)
{
    ::test::payload_pool_performance_templatized_test<core::MapPayloadPool>();
}

TEST(PayloadPoolPerformanceTest, fast_payload_pool)
{
    ::test::payload_pool_performance_templatized_test<core::FastPayloadPool>();
}

int main(
        int argc,
        char** argv)
{
    // utils::Log::SetVerbosity(utils::Log::Kind::Info);
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

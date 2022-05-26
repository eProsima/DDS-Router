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
#include <thread>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <TestLogHandler.hpp>

#include <fastdds/rtps/common/SerializedPayload.h>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/time/Timer.hpp>
#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <efficiency/MapPayloadPool.hpp>
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

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace test {

constexpr const uint32_t REPETITION_TEST = 100;

void execute_performance_test(
    std::ostream& output_stream,
    const std::vector<std::string>& column_names,
    const uint32_t test_repetitions,
    const std::vector<std::vector<uint32_t>>& arguments,
    const std::function<double (const std::vector<uint32_t>&)>& test_function,
    const std::string& separator = ",")
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

double communication_mock_performance_test_(
        unsigned int data_size,
        unsigned int samples_to_send)
{
    // Set constant values
    std::string mock_participant_name_1 = "ParticipantMock1";
    std::string mock_participant_name_2 = "ParticipantMock2";
    std::string topic_name_1 = "Topic1";
    std::string topic_type_1 = "TopicType1";

    // Create Router configuration
    core::configuration::DDSRouterConfiguration configuration(
        std::set<std::shared_ptr<types::FilterTopic>>(),
        std::set<std::shared_ptr<types::FilterTopic>>(),
        std::set<std::shared_ptr<types::RealTopic>>(
                {
                    std::make_shared<types::RealTopic>(
                        topic_name_1,
                        topic_type_1
                    ),
                }
            ),
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>>(
                {
                    std::make_shared<configuration::ParticipantConfiguration>(
                        types::ParticipantId(mock_participant_name_1),
                        types::ParticipantKind::MOCK
                        ),
                    std::make_shared<configuration::ParticipantConfiguration>(
                        types::ParticipantId(mock_participant_name_2),
                        types::ParticipantKind::MOCK
                        ),
                }
            ));

    // Start Router
    core::DDSRouter router(configuration);
    router.start();

    // From here router is created and initialized
    // Get Mock Participant
    MockParticipant* part1 = MockParticipant::get_participant(mock_participant_name_1);
    MockParticipant* part2 = MockParticipant::get_participant(mock_participant_name_2);
    types::RealTopic topic_1(topic_name_1, topic_type_1);

    // Get writer and reader from different participants to test performance
    MockReader* reader1 = part1->get_reader(topic_1);
    MockWriter* writer2 = part2->get_writer(topic_1);

    // Prepare sending data
    types::Guid simulated_writer_guid;
    std::string str_data(data_size, 'x');
    void* raw_data = static_cast<void*>((char*)str_data.c_str());

    // Start Timer
    utils::Timer timer;

    // Send data from readers and wait for them in writers
    for (uint32_t i = 0; i < samples_to_send; ++i)
    {
        // Simulate to send this data
        reader1->simulate_data_reception(simulated_writer_guid, static_cast<void*>((char*)raw_data), data_size);
    }

    writer2->wait_until_n_data_sent(
        samples_to_send);

    // Stop timer
    double result = timer.elapsed();

    router.stop();

    return result;
}

double communication_mock_performance_test(
        std::vector<uint32_t> arguments)
{
    // Get arguments
    uint32_t data_size = arguments[0];
    uint32_t samples_to_send = arguments[1];

    return communication_mock_performance_test_(data_size, samples_to_send);
}

template <class PP>
double memcpy_payload_performance_test_(
        unsigned int data_size,
        unsigned int samples_to_send)
{
    std::string str_data(data_size, 'x');
    void* raw_data = static_cast<void*>((char*)str_data.c_str());

    PP payload_pool;
    eprosima::ddsrouter::core::types::Payload payload;

    // Start Timer
    utils::Timer timer;

    for (uint32_t i = 0; i < samples_to_send; ++i)
    {
        // Simulate to send this data
        payload_pool.get_payload(data_size, payload);
        // Copy memory
        std::memcpy(payload.data, raw_data, data_size);
    }

    // Stop timer
    double result = timer.elapsed();

    return result;
}

template <class PP>
double memcpy_payload_performance_test(
        std::vector<uint32_t> arguments)
{
    // Get arguments
    uint32_t data_size = arguments[0];
    uint32_t samples_to_send = arguments[1];

    return memcpy_payload_performance_test_<PP>(data_size, samples_to_send);
}

} /* namespace test */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

TEST(CommunicationMockPerformanceTest, test_all)
{
    // Prepare column names
    std::vector<std::string> column_names{
        "Data Size",
        "Number of Samples"
    };

    // Prepare argument vectors;
    std::vector<uint32_t> data_sizes{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288, 1048576, 2097152};
    std::vector<uint32_t> number_of_samples{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

    // Launch test and print results in stdout
    eprosima::ddsrouter::core::test::execute_performance_test(
        std::cout,
        column_names,
        eprosima::ddsrouter::core::test::REPETITION_TEST,
        std::vector<std::vector<uint32_t>>{data_sizes, number_of_samples},
        eprosima::ddsrouter::core::test::communication_mock_performance_test);
}

TEST(CommunicationMockPerformanceTest, mem_copy)
{
    // Prepare column names
    std::vector<std::string> column_names{
        "Data Size",
        "Number of Samples"
    };

    // Prepare argument vectors;
    std::vector<uint32_t> data_sizes{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288, 1048576, 2097152};
    std::vector<uint32_t> number_of_samples{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

    // Launch test and print results in stdout
    eprosima::ddsrouter::core::test::execute_performance_test(
        std::cout,
        column_names,
        eprosima::ddsrouter::core::test::REPETITION_TEST,
        std::vector<std::vector<uint32_t>>{data_sizes, number_of_samples},
        eprosima::ddsrouter::core::test::memcpy_payload_performance_test<eprosima::ddsrouter::core::MapPayloadPool>);
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

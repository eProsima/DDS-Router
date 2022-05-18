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

constexpr const uint32_t DEFAULT_SAMPLE_SIZE(32);
constexpr const uint32_t DEFAULT_SAMPLES_TO_SEND(10e5);

} /* namespace test */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

TEST(MockPerformanceTest, dummy_test)
{
    std::string mock_participant_name_1 = "ParticipantMock1";
    std::string mock_participant_name_2 = "ParticipantMock2";
    std::string topic_name_1 = "Topic1";
    std::string topic_type_1 = "TopicType1";

    core::configuration::DDSRouterConfiguration configuration(
        std::set<std::shared_ptr<FilterTopic>>(),
        std::set<std::shared_ptr<FilterTopic>>(),
        std::set<std::shared_ptr<RealTopic>>(
                {
                    std::make_shared<RealTopic>(
                        topic_name_1,
                        topic_type_1
                    ),
                }
            ),
        std::set<std::shared_ptr<configuration::ParticipantConfiguration>>(
                {
                    std::make_shared<configuration::ParticipantConfiguration>(
                        ParticipantId(mock_participant_name_1),
                        ParticipantKind::MOCK
                        ),
                    std::make_shared<configuration::ParticipantConfiguration>(
                        ParticipantId(mock_participant_name_2),
                        ParticipantKind::MOCK
                        ),
                }
            ));

    core::DDSRouter router(configuration);
    router.start();

    // From here router is created and initialized
    // Get Mock Participant
    MockParticipant* part1 = MockParticipant::get_participant(mock_participant_name_1);
    MockParticipant* part2 = MockParticipant::get_participant(mock_participant_name_2);
    RealTopic topic_1(topic_name_1, topic_type_1);

    // Get writer and reader from different participants to test performance
    MockReader* reader1 = part1->get_reader(topic_1);
    MockWriter* writer2 = part2->get_writer(topic_1);

    // Prepare sending data
    types::Guid simulated_writer_guid;
    const char* raw_data = "0123456789";
    uint32_t size = 10;

    // Initialize timer
    utils::Timer timer;

    for (uint32_t i = 0; i < core::test::DEFAULT_SAMPLES_TO_SEND; ++i)
    {
        // Simulate to send this data
        reader1->simulate_data_reception(simulated_writer_guid, static_cast<void*>((char*)raw_data), size);
    }

    writer2->wait_until_n_data_sent(
        core::test::DEFAULT_SAMPLES_TO_SEND);

    auto time_elapsed = timer.elapsed();

    logUser(PERFORMANCE_TEST, core::test::DEFAULT_SAMPLES_TO_SEND << ";" << time_elapsed);

    router.stop();
}

int main(
        int argc,
        char** argv)
{
    // utils::Log::SetVerbosity(utils::Log::Kind::Info);
    utils::Log::Flush();
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

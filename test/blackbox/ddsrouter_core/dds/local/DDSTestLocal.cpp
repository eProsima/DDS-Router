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

#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/types/Log.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

#include <test_participants.hpp>

using namespace eprosima::ddsrouter;

constexpr const uint32_t SAMPLES_TO_RECEIVE = 5;
constexpr const uint32_t MILLISECONDS_PUBLISH_LOOP = 100;

/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
 */
template <class MsgStruct>
void test_local_communication(
        std::string config_path)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    test::TestLogHandler test_log_handler(Log::Kind::Error);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    MsgStruct msg;
    msg.message("Testing DDS-Router Blackbox Local...");

    // Create DDS Publisher in domain 0
    HelloWorldPublisher<MsgStruct> publisher(msg.isKeyDefined());
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    HelloWorldSubscriber<MsgStruct> subscriber(msg.isKeyDefined());
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Load configuration containing two Simple Participants, one in domain 0 and another one in domain 1
    RawConfiguration router_configuration =
            load_configuration_from_file(config_path);

    // Create DDSRouter entity
    DDSRouter router(router_configuration);
    router.start();

    // Start publishing
    while (samples_received.load() < SAMPLES_TO_RECEIVE)
    {
        msg.index(++samples_sent);
        publisher.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(MILLISECONDS_PUBLISH_LOOP));
    }

    router.stop();
}

/**
 * Test whole DDSRouter initialization by initializing two Simple Participants
 */
TEST(DDSTestLocal, simple_initialization)
{
    // Load configuration
    RawConfiguration router_configuration =
            load_configuration_from_file("../../resources/configurations/dds/local/dds_test_simple_configuration.yaml");

    // Create DDSRouter entity
    DDSRouter router(router_configuration);

    // Let test finish without failing
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication)
{
    test_local_communication<HelloWorld>("../../resources/configurations/dds/local/dds_test_simple_configuration.yaml");
}

/**
 * Test communication in HelloWorldKeyed topic between two DDS participants created in different domains,
 * by using a router with two Simple Participants at each domain.
 */
TEST(DDSTestLocal, end_to_end_local_communication_keyed)
{
    test_local_communication<HelloWorldKeyed>(
        "../../resources/configurations/dds/local/dds_test_simple_configuration.yaml");
}


int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

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
#include <mutex>
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
 * This is accomplished by connecting two WAN Participants belonging to different DDS Router instances. These router
 * instances communicate with the DDS Participants through Simple Participants deployed at those domains.
 */
void test_WAN_communication(
        std::string server_config_path,
        std::string client_config_path)
{
    // Check there are no warnings/errors
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    test::TestLogHandler test_log_handler(Log::Kind::Error);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    HelloWorld msg;
    msg.message("Testing DDS-Router Blackbox WAN...");

    // Create DDS Publisher in domain 0
    HelloWorldPublisher<HelloWorld> publisher;
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    HelloWorldSubscriber<HelloWorld> subscriber;
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Load configuration containing a Simple Participant in domain 0 and a WAN Participant configured as server
    // (possibly also as client)
    RawConfiguration server_router_configuration =
            load_configuration_from_file(server_config_path);

    // Load configuration containing a Simple Participant in domain 1 and a WAN Participant configured as client
    // (possibly also as server)
    RawConfiguration client_router_configuration =
            load_configuration_from_file(client_config_path);

    // Create DDSRouter entity whose WAN Participant is configured as server
    DDSRouter server_router(server_router_configuration);
    server_router.start();

    // Create DDSRouter entity whose WAN Participant is configured as client
    DDSRouter client_router(client_router_configuration);
    client_router.start();

    // Start publishing
    while (samples_received.load() < SAMPLES_TO_RECEIVE)
    {
        msg.index(++samples_sent);
        publisher.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(MILLISECONDS_PUBLISH_LOOP));
    }

    client_router.stop();
    server_router.stop();
}

/**
 * Test communication between WAN participants by running \c test_WAN_communication for different configurations.
 * Different combinations of server/client configurations are tested.
 *
 * CASES:
 *  server <-> client
 *  server <-> server-client
 *  server-client <-> server-client
 */
void test_WAN_communication_all(
        std::string dir_path,
        bool basic_only = false)
{
    // server <-> client
    test_WAN_communication(dir_path + "server.yaml", dir_path + "client.yaml");

    // server <-> server-client
    test_WAN_communication(dir_path + "server.yaml", dir_path + "server-client-A.yaml");

    // This test is disabled for TCPv6 and TLSv6, as an underlying middleware issue resulting in no matching for this
    // scenario exists.
    // TODO: Test this as well for TCPv6 and TLSv6 cases when issue is fixed
    if (!basic_only)
    {
        // server-client <-> server-client
        test_WAN_communication(dir_path + "server-client-B.yaml", dir_path + "server-client-A.yaml");
    }
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv4.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_UDPv4)
{
    test_WAN_communication_all("../../resources/configurations/dds/WAN/UDP/IPv4/");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv6.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_UDPv6)
{
    test_WAN_communication_all("../../resources/configurations/dds/WAN/UDP/IPv6/");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TCPv4.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TCPv4)
{
    test_WAN_communication_all("../../resources/configurations/dds/WAN/TCP/IPv4/");
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TCPv6.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TCPv6)
{
    test_WAN_communication_all("../../resources/configurations/dds/WAN/TCP/IPv6/", true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TLSv4.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TLSv4)
{
    test_WAN_communication_all("../../resources/configurations/dds/WAN/TLS/IPv4/", false);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TLSv6.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TLSv6)
{
    test_WAN_communication_all("../../resources/configurations/dds/WAN/TLS/IPv6/", true);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

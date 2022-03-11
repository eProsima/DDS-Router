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

#include <ddsrouter_core/configuration/participant/DiscoveryServerParticipantConfiguration.hpp>
#include <ddsrouter_core/core/DDSRouter.hpp>
#include <ddsrouter_core/types/address/Address.hpp>
#include <ddsrouter_core/types/security/tls/TlsConfigurationBoth.hpp>
#include <ddsrouter_core/types/security/tls/TlsConfigurationServer.hpp>
#include <ddsrouter_core/types/topic/WildcardTopic.hpp>

#include <test_participants.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace test {

enum WanKind
{
    SERVER,
    CLIENT,
    SERVER_AND_CLIENT
};

bool is_client(
        WanKind wan_kind)
{
    return wan_kind == CLIENT || wan_kind == SERVER_AND_CLIENT;
}

bool is_server(
        WanKind wan_kind)
{
    return wan_kind == SERVER || wan_kind == SERVER_AND_CLIENT;
}

constexpr const uint32_t DEFAULT_SAMPLES_TO_RECEIVE = 5;
constexpr const uint32_t DEFAULT_MILLISECONDS_PUBLISH_LOOP = 100;
constexpr const uint32_t DEFAULT_MESSAGE_SIZE = 1; // x50 bytes

std::shared_ptr<types::security::TlsConfiguration> tls_configuration(
        WanKind wan_kind)
{
    // TODO: test that using server with only Server required files works
    // It fails when connecting to other server
    if (is_server(wan_kind))
    {
        return std::make_shared<types::security::TlsConfigurationBoth>(
            "../../resources/tls/ca.crt", // ca
            "", // private key password
            "../../resources/tls/ddsrouter.key", // private key
            "../../resources/tls/ddsrouter.crt", // cert
            "../../resources/tls/dh_params.pem" // dh params
            );
    }
    else
    {
        return std::make_shared<types::security::TlsConfigurationClient>(
            "../../resources/tls/ca.crt" // ca
            );
    }
}

std::shared_ptr<configuration::ParticipantConfiguration> wan_participant_configuration(
        bool this_server_id_is_1,
        WanKind wan_kind,
        types::TransportProtocol transport_protocol,
        types::IpVersion ip_version,
        bool tls = false)
{
    std::set<types::Address> listening_addresses;
    std::set<types::DiscoveryServerConnectionAddress> connection_addresses;

    if (is_client(wan_kind))
    {
        connection_addresses.insert(
            types::DiscoveryServerConnectionAddress(
                types::GuidPrefix((this_server_id_is_1 ? 0u : 1u)),
                        {
                            types::Address(
                                (ip_version == types::IpVersion::IPv4 ? "127.0.0.1" : "::1"),
                                11666 + (this_server_id_is_1 ? 0u : 1u),
                                ip_version,
                                transport_protocol)
                        }
                )
            );
    }

    if (is_server(wan_kind))
    {
        listening_addresses.insert(
            types::Address(
                (ip_version == types::IpVersion::IPv4 ? "127.0.0.1" : "::1"),
                11666 + (this_server_id_is_1 ? 1u : 0u),
                ip_version,
                transport_protocol)
            );
    }

    if (tls)
    {
        return std::make_shared<configuration::DiscoveryServerParticipantConfiguration>(
            types::ParticipantId("WanParticipant_" + std::to_string((this_server_id_is_1 ? 1 : 0))),
            types::GuidPrefix((this_server_id_is_1 ? 1u : 0u)),
            listening_addresses,
            connection_addresses,
            types::ParticipantKind(types::ParticipantKind::WAN),
            tls_configuration(wan_kind));

    }
    else
    {
        return std::make_shared<configuration::DiscoveryServerParticipantConfiguration>(
            types::ParticipantId("WanParticipant_" + std::to_string((this_server_id_is_1 ? 1 : 0))),
            types::GuidPrefix((this_server_id_is_1 ? 1u : 0u)),
            listening_addresses,
            connection_addresses,
            types::ParticipantKind(types::ParticipantKind::WAN)
            );
    }
}

/**
 * @brief Create a simple configuration for a DDS Router
 *
 * Create a configuration with 1 topic
 * Create 1 simple participants with domains \c domain
 * Create 1 custom participant by the configuration in \c participant_configuration
 *
 * @return configuration::DDSRouterConfiguration
 */
configuration::DDSRouterConfiguration router_configuration(
        std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
        types::DomainIdType domain)
{
    // One topic
    std::set<std::shared_ptr<types::FilterTopic>> allowlist(
                    {
                        std::make_shared<types::WildcardTopic>("DDS-Router-Test", "HelloWorld"),
                    });
    std::set<std::shared_ptr<types::FilterTopic>> blocklist;   // empty

    std::set<std::shared_ptr<types::RealTopic>> builtin_topics;   // empty

    // Two participants, one custom and other simple. If server, simple will work in 0, if not in 1
    std::set<std::shared_ptr<configuration::ParticipantConfiguration>> participants_configurations(
                    {
                        // custom
                        participant_configuration,

                        // simple
                        std::make_shared<configuration::SimpleParticipantConfiguration>(
                            types::ParticipantId("simple_participant"),
                            types::ParticipantKind(types::ParticipantKind::SIMPLE_RTPS),
                            types::DomainId(domain)
                            ),
                    }
        );

    return configuration::DDSRouterConfiguration(
        allowlist,
        blocklist,
        builtin_topics,
        participants_configurations
        );
}

/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by connecting two WAN Participants belonging to different DDS Router instances. These router
 * instances communicate with the DDS Participants through Simple Participants deployed at those domains.
 */
void test_WAN_communication(
        configuration::DDSRouterConfiguration ddsrouter_server_configuration,
        configuration::DDSRouterConfiguration ddsrouter_client_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Uncomment when having no listening addresses is no longer considered an error by the middleware
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // test::TestLogHandler test_log_handler(Log::Kind::Error);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorld msg;
    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DDSRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorld> publisher;
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorld> subscriber;
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DDSRouter entity whose WAN Participant is configured as server
    DDSRouter server_router(ddsrouter_server_configuration);
    server_router.start();

    // Create DDSRouter entity whose WAN Participant is configured as client
    DDSRouter client_router(ddsrouter_client_configuration);
    client_router.start();

    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        publisher.publish(msg);

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
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
 *  server-client <-> server-client (only when basic_only is deactivate)
 */
void test_WAN_communication_all(
        types::TransportProtocol transport_protocol,
        types::IpVersion ip_version,
        bool basic_only = false,
        bool tls = false)
{
    // Test architecture server <-> client
    test::test_WAN_communication(
        test::router_configuration(
            test::wan_participant_configuration(
                true, // is server 1
                SERVER,
                transport_protocol, // transport protocol
                ip_version, // ip version
                tls // tls
                ),
            0 // domain
            ),

        test::router_configuration(
            test::wan_participant_configuration(
                false, // is server 1
                CLIENT,
                transport_protocol, // transport protocol
                ip_version, // ip version
                tls // tls
                ),
            1 // domain
            )
        );

    // Test architecture server <-> server-client
    test::test_WAN_communication(
        test::router_configuration(
            test::wan_participant_configuration(
                true, // is server 1
                SERVER,
                transport_protocol, // transport protocol
                ip_version, // ip version
                tls // tls
                ),
            0 // domain
            ),

        test::router_configuration(
            test::wan_participant_configuration(
                false, // is server 1
                SERVER_AND_CLIENT,
                transport_protocol, // transport protocol
                ip_version, // ip version
                tls // tls
                ),
            1 // domain
            )
        );

    // This test is disabled for TCPv6 and TLSv6, as an underlying middleware issue resulting in no matching for this
    // scenario exists.
    // TODO: Test this as well for TCPv6 and TLSv6 cases when issue is fixed
    if (!basic_only)
    {
        // Test architecture server-client <-> server-client
        test::test_WAN_communication(
            test::router_configuration(
                test::wan_participant_configuration(
                    true, // is server 1
                    SERVER_AND_CLIENT,
                    transport_protocol, // transport protocol
                    ip_version, // ip version
                    tls // tls
                    ),
                0 // domain
                ),

            test::router_configuration(
                test::wan_participant_configuration(
                    false, // is server 1
                    SERVER_AND_CLIENT,
                    transport_protocol, // transport protocol
                    ip_version, // ip version
                    tls // tls
                    ),
                1 // domain
                )
            );
    }
}

} /* namespace test */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter::core;
using namespace eprosima::ddsrouter::core::types;

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv4.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_UDPv4)
{
    test::test_WAN_communication_all(
        types::TransportProtocol::UDP,
        types::IpVersion::IPv4);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv6.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_UDPv6)
{
    test::test_WAN_communication_all(
        types::TransportProtocol::UDP,
        types::IpVersion::IPv6);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TCPv4.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TCPv4)
{
    test::test_WAN_communication_all(
        types::TransportProtocol::TCP,
        types::IpVersion::IPv4);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TCPv6.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TCPv6)
{
    test::test_WAN_communication_all(
        types::TransportProtocol::TCP,
        types::IpVersion::IPv6,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TLSv4.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TLSv4)
{
    test::test_WAN_communication_all(
        types::TransportProtocol::TCP,
        types::IpVersion::IPv4,
        false,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through TLSv6.
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_TLSv6)
{
    test::test_WAN_communication_all(
        types::TransportProtocol::TCP,
        types::IpVersion::IPv6,
        true,
        true);
}

/**
 * Test high throughput communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two WAN Participants connected
 * through UDPv4.
 *
 * PARAMETERS:
 * - Frequency: 1ms
 * - Sample size: 50K
 * -> Throughput: 50MBps
 */
TEST(DDSTestWAN, end_to_end_WAN_communication_high_throughput)
{
    test::test_WAN_communication(
        test::router_configuration(
            test::wan_participant_configuration(
                true, // is server 1
                test::SERVER,
                types::TransportProtocol::UDP, // transport protocol
                types::IpVersion::IPv4 // ip version
                ),
            0 // domain
            ),

        test::router_configuration(
            test::wan_participant_configuration(
                false, // is server 1
                test::CLIENT,
                types::TransportProtocol::UDP, // transport protocol
                types::IpVersion::IPv4 // ip version
                ),
            1 // domain
            ),

        500,
        1,
        1000); // 50K message size
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

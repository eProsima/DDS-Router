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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/types/address/Address.hpp>
#include <ddspipe_participants/types/security/tls/TlsConfiguration.hpp>

#include <ddsrouter_core/core/DdsRouter.hpp>

#include <test_participants.hpp>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddsrouter::core;

namespace test {

enum class WanParticipantKind
{
    discovery_server,
    initial_peers
};

enum class WanKind
{
    server,
    client,
    server_and_client
};

bool is_client(
        WanKind wan_kind)
{
    return wan_kind == WanKind::client || wan_kind == WanKind::server_and_client;
}

bool is_server(
        WanKind wan_kind)
{
    return wan_kind == WanKind::server || wan_kind == WanKind::server_and_client;
}

constexpr const uint32_t DEFAULT_SAMPLES_TO_RECEIVE = 5;
constexpr const uint32_t DEFAULT_MILLISECONDS_PUBLISH_LOOP = 100;
constexpr const uint32_t DEFAULT_MESSAGE_SIZE = 1; // x50 bytes

participants::types::TlsConfiguration tls_configuration(
        WanKind wan_kind)
{
    // TODO: test that using server with only Server required files works
    // It fails when connecting to other server
    if (is_server(wan_kind))
    {
        participants::types::TlsConfiguration tls;
        tls.certificate_authority_file = "../../resources/tls/ca.crt";
        tls.private_key_file = "../../resources/tls/ddsrouter.key";
        tls.certificate_chain_file = "../../resources/tls/ddsrouter.crt";
        tls.dh_params_file = "../../resources/tls/dh_params.pem";
        return tls;
    }
    else
    {
        participants::types::TlsConfiguration tls;
        tls.certificate_authority_file = "../../resources/tls/ca.crt";
        return tls;
    }
}

std::pair<
    types::ParticipantKind,
    std::shared_ptr<participants::ParticipantConfiguration>>
discovery_server_participant_configuration(
        bool this_server_id_is_1,
        WanKind wan_kind,
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version,
        bool tls = false)
{
    participants::DiscoveryServerParticipantConfiguration conf;

    if (is_client(wan_kind))
    {
        conf.connection_addresses.insert(
            participants::types::DiscoveryServerConnectionAddress(
                core::types::GuidPrefix((this_server_id_is_1 ? 0u : 1u)),
            {
                participants::types::Address(
                    (ip_version == participants::types::IpVersion::v4 ? "127.0.0.1" : "::1"),
                    11666 + (this_server_id_is_1 ? 0u : 1u),
                    11666 + (this_server_id_is_1 ? 0u : 1u),
                    ip_version,
                    transport_protocol)
            }
                )
            );
    }

    if (is_server(wan_kind))
    {
        conf.listening_addresses.insert(
            participants::types::Address(
                (ip_version == participants::types::IpVersion::v4 ? "127.0.0.1" : "::1"),
                11666 + (this_server_id_is_1 ? 1u : 0u),
                11666 + (this_server_id_is_1 ? 1u : 0u),
                ip_version,
                transport_protocol)
            );
    }

    conf.id = core::types::ParticipantId("WanDsParticipant_" + std::to_string((this_server_id_is_1 ? 1 : 0)));

    conf.discovery_server_guid_prefix = core::types::GuidPrefix((this_server_id_is_1 ? 1u : 0u));

    if (tls)
    {
        conf.tls_configuration = tls_configuration(wan_kind);
    }

    return {
        types::ParticipantKind::discovery_server,
        std::make_shared<participants::DiscoveryServerParticipantConfiguration>(conf)
    };
}

std::pair<
    types::ParticipantKind,
    std::shared_ptr<participants::ParticipantConfiguration>>
initial_peers_participant_configuration(
        bool this_server_id_is_1,
        WanKind wan_kind,
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version,
        bool tls = false)
{
    participants::InitialPeersParticipantConfiguration conf;

    if (is_client(wan_kind))
    {
        conf.connection_addresses.insert(
            participants::types::Address(
                (ip_version == participants::types::IpVersion::v4 ? "127.0.0.1" : "::1"),
                11666 + (this_server_id_is_1 ? 0u : 1u),
                11666 + (this_server_id_is_1 ? 0u : 1u),
                ip_version,
                transport_protocol)
            );
    }

    if (is_server(wan_kind))
    {
        conf.listening_addresses.insert(
            participants::types::Address(
                (ip_version == participants::types::IpVersion::v4 ? "127.0.0.1" : "::1"),
                11666 + (this_server_id_is_1 ? 1u : 0u),
                11666 + (this_server_id_is_1 ? 1u : 0u),
                ip_version,
                transport_protocol)
            );
    }

    conf.id = core::types::ParticipantId("WanDsParticipant_" + std::to_string((this_server_id_is_1 ? 1 : 0)));

    if (tls)
    {
        conf.tls_configuration = tls_configuration(wan_kind);
    }

    return {
        types::ParticipantKind::initial_peers,
        std::make_shared<participants::InitialPeersParticipantConfiguration>(conf)
    };
}

std::pair<
    types::ParticipantKind,
    std::shared_ptr<participants::ParticipantConfiguration>>
wan_participant_configuration(
        WanParticipantKind wan_participant_kind,
        bool this_server_id_is_1,
        WanKind wan_kind,
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version,
        bool tls = false)
{
    if (wan_participant_kind == WanParticipantKind::discovery_server)
    {
        return discovery_server_participant_configuration(this_server_id_is_1, wan_kind, transport_protocol, ip_version,
                       tls);
    }
    else // WanParticipantKind::initial_peers
    {
        return initial_peers_participant_configuration(this_server_id_is_1, wan_kind, transport_protocol, ip_version,
                       tls);
    }
}

/**
 * @brief Create a simple configuration for a DDS Router
 *
 * Create a configuration with 1 topic
 * Create 1 simple participants with domains \c domain
 * Create 1 custom participant by the configuration in \c participant_configuration
 *
 * @return DdsRouterConfiguration
 */
DdsRouterConfiguration router_configuration(
        std::pair<
            types::ParticipantKind,
            std::shared_ptr<participants::ParticipantConfiguration>>
        participant_configuration,
        core::types::DomainIdType domain)
{
    DdsRouterConfiguration conf;

    // One topic
    core::types::WildcardDdsFilterTopic topic;
    topic.topic_name.set_value(TOPIC_NAME);
    conf.ddspipe_configuration.allowlist.insert(
        utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(topic));

    // Two participants, one custom and other simple. If server, simple will work in 0, if not in 1
    conf.participants_configurations.insert(participant_configuration);
    {
        auto part = std::make_shared<participants::SimpleParticipantConfiguration>();
        part->id = core::types::ParticipantId("simple_participant_" + std::to_string(domain));
        part->domain.domain_id = domain;
        conf.participants_configurations.insert({types::ParticipantKind::simple, part});
    }

    return conf;
}

/**
 * Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
 * This is accomplished by connecting two WAN Participants belonging to different DDS Router instances. These router
 * instances communicate with the DDS Participants through Simple Participants deployed at those domains.
 */
void test_WAN_communication(
        DdsRouterConfiguration ddsrouter_server_configuration,
        DdsRouterConfiguration ddsrouter_client_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    // Check there are no warnings/errors
    // TODO: Uncomment when having no listening addresses is no longer considered an error by the middleware
    // TODO: Change threshold to \c Log::Kind::Warning once middleware warnings are solved
    // test::LogChecker test_log_handler(Log::Kind::Error);

    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    // Create a message with size specified by repeating the same string
    HelloWorld msg;
    std::string msg_str;

    // Add this string as many times as the msg size requires
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Local Communication ...";
    }
    msg.message(msg_str);

    // Create DDS Publisher in domain 0
    TestPublisher<HelloWorld> publisher;
    ASSERT_TRUE(publisher.init(0));

    // Create DDS Subscriber in domain 1
    TestSubscriber<HelloWorld> subscriber;
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create DdsRouter entity whose WAN Participant is configured as server
    DdsRouter server_router(ddsrouter_server_configuration);
    server_router.start();

    // Create DdsRouter entity whose WAN Participant is configured as client
    DdsRouter client_router(ddsrouter_client_configuration);
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
        WanParticipantKind wan_participant_kind,
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version,
        bool basic_only = false,
        bool tls = false)
{
    // Test architecture server <-> client
    test::test_WAN_communication(
        test::router_configuration(
            test::wan_participant_configuration(
                wan_participant_kind,
                true, // is server 1
                WanKind::server,
                transport_protocol, // transport protocol
                ip_version, // ip version
                tls // tls
                ),
            0 // domain
            ),

        test::router_configuration(
            test::wan_participant_configuration(
                wan_participant_kind,
                false, // is server 1
                WanKind::client,
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
                wan_participant_kind,
                true, // is server 1
                WanKind::server,
                transport_protocol, // transport protocol
                ip_version, // ip version
                tls // tls
                ),
            0 // domain
            ),

        test::router_configuration(
            test::wan_participant_configuration(
                wan_participant_kind,
                false, // is server 1
                WanKind::server_and_client,
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
                    wan_participant_kind,
                    true, // is server 1
                    WanKind::server_and_client,
                    transport_protocol, // transport protocol
                    ip_version, // ip version
                    tls // tls
                    ),
                0 // domain
                ),

            test::router_configuration(
                test::wan_participant_configuration(
                    wan_participant_kind,
                    false, // is server 1
                    WanKind::server_and_client,
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

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through UDPv4.
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_UDPv4)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::discovery_server,
        participants::types::TransportProtocol::udp,
        participants::types::IpVersion::v4);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through UDPv4.
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_UDPv4)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::initial_peers,
        participants::types::TransportProtocol::udp,
        participants::types::IpVersion::v4);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through UDPv6.
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_UDPv6)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::discovery_server,
        participants::types::TransportProtocol::udp,
        participants::types::IpVersion::v6);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through UDPv6.
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_UDPv6)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::initial_peers,
        participants::types::TransportProtocol::udp,
        participants::types::IpVersion::v6);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through TCPv4.
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_TCPv4)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::discovery_server,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v4);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through TCPv4.
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_TCPv4)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::initial_peers,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v4);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through TCPv6.
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_TCPv6)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::discovery_server,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v6,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through TCPv6.
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_TCPv6)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::initial_peers,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v6,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through TLSv4.
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_TLSv4)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::discovery_server,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v4,
        false,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through TLSv4.
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_TLSv4)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::initial_peers,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v4,
        false,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through TLSv6.
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_TLSv6)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::discovery_server,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v6,
        true,
        true);
}

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through TLSv6.
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_TLSv6)
{
    test::test_WAN_communication_all(
        test::WanParticipantKind::initial_peers,
        participants::types::TransportProtocol::tcp,
        participants::types::IpVersion::v6,
        true,
        true);
}

/**
 * Test high throughput communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through UDPv4.
 *
 * PARAMETERS:
 * - Frequency: 1ms
 * - Sample size: 50K
 * -> Throughput: 50MBps
 */
TEST(DDSTestWAN, end_to_end_discovery_server_WAN_communication_high_throughput)
{
    test::test_WAN_communication(
        test::router_configuration(
            test::discovery_server_participant_configuration(
                true, // is server 1
                test::WanKind::server,
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
                ),
            0 // domain
            ),

        test::router_configuration(
            test::discovery_server_participant_configuration(
                false, // is server 1
                test::WanKind::client,
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
                ),
            1 // domain
            ),

        500,
        1,
        1000); // 50K message size
}

/**
 * Test high throughput communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Initial Peers Participants connected
 * through UDPv4.
 *
 * PARAMETERS:
 * - Frequency: 1ms
 * - Sample size: 50K
 * -> Throughput: 50MBps
 */
TEST(DDSTestWAN, end_to_end_initial_peers_WAN_communication_high_throughput)
{
    test::test_WAN_communication(
        test::router_configuration(
            test::initial_peers_participant_configuration(
                true, // is server 1
                test::WanKind::server,
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
                ),
            0 // domain
            ),

        test::router_configuration(
            test::initial_peers_participant_configuration(
                false, // is server 1
                test::WanKind::client,
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
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

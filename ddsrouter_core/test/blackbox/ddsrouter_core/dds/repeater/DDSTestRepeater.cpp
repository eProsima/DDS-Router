// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

constexpr const uint32_t DEFAULT_SAMPLES_TO_RECEIVE = 5;
constexpr const uint32_t DEFAULT_MILLISECONDS_PUBLISH_LOOP = 100;
constexpr const uint32_t DEFAULT_MESSAGE_SIZE = 1; // x50 bytes

std::pair<
    types::ParticipantKind,
    std::shared_ptr<participants::ParticipantConfiguration>>
initial_peers_participant_configuration_client(
        bool this_server_id_is_1,
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version)
{
    auto conf = std::make_shared<participants::InitialPeersParticipantConfiguration>();

    conf->id =
            core::types::ParticipantId("InitialPeersParticipant_Client_" +
                    std::to_string((this_server_id_is_1 ? 1 : 0)));
    conf->connection_addresses.insert(
        participants::types::Address(
            (ip_version == participants::types::IpVersion::v4 ? "127.0.0.1" : "::1"),
            11666,
            11666,
            ip_version,
            transport_protocol)
        );

    return {types::ParticipantKind::initial_peers, conf};
}

std::pair<
    types::ParticipantKind,
    std::shared_ptr<participants::ParticipantConfiguration>>
initial_peers_participant_configuration_server(
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version)
{
    auto conf = std::make_shared<participants::InitialPeersParticipantConfiguration>();

    conf->id = core::types::ParticipantId("InitialPeersParticipant_Server");
    conf->listening_addresses.insert(
        participants::types::Address(
            (ip_version == participants::types::IpVersion::v4 ? "127.0.0.1" : "::1"),
            11666,
            11666,
            ip_version,
            transport_protocol)
        );
    conf->is_repeater = true;

    return {types::ParticipantKind::initial_peers, conf};
}

std::pair<
    types::ParticipantKind,
    std::shared_ptr<participants::ParticipantConfiguration>>
participant_configuration(
        bool server,
        bool this_server_id_is_1,
        participants::types::TransportProtocol transport_protocol,
        participants::types::IpVersion ip_version)
{
    if (server)
    {
        return initial_peers_participant_configuration_server(transport_protocol, ip_version);
    }
    else
    {
        return initial_peers_participant_configuration_client(this_server_id_is_1, transport_protocol, ip_version);
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
        core::types::DomainIdType domain,
        bool has_simple = true)
{
    DdsRouterConfiguration conf;

    // One topic
    core::types::WildcardDdsFilterTopic topic;
    topic.topic_name.set_value(TOPIC_NAME);
    conf.ddspipe_configuration.allowlist.insert(
        utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(topic));

    // Two participants, one custom and other simple. If server, simple will work in 0, if not in 1
    conf.participants_configurations.insert(participant_configuration);
    if (has_simple)
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
        DdsRouterConfiguration ddsrouter_client_configuration_1,
        DdsRouterConfiguration ddsrouter_client_configuration_2,
        DdsRouterConfiguration ddsrouter_repeater_configuration,
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
    DdsRouter client_router_1(ddsrouter_client_configuration_1);
    client_router_1.start();

    // Create DdsRouter entity whose WAN Participant is configured as client
    DdsRouter client_router_2(ddsrouter_client_configuration_2);
    client_router_2.start();

    // Create DdsRouter entity whose WAN Participant is configured as repeater
    DdsRouter repeater(ddsrouter_repeater_configuration);
    repeater.start();

    // Start publishing
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        publisher.publish(msg);

        // If time is 0 do not wait
        if (time_between_samples > 0)
        {
            eprosima::utils::sleep_for(time_between_samples);
        }
    }

    client_router_1.stop();
    client_router_2.stop();
    repeater.stop();
}

} /* namespace test */

/**
 * Test communication in HelloWorld topic between two DDS participants created in different domains,
 * by using two routers with two Simple Participants at each domain, and two Discovery Server Participants connected
 * through UDPv4.
 */
TEST(DDSTestRepeater, repeater_initial_peers_communication_UDPv4)
{
    test::test_WAN_communication(

        test::router_configuration(
            test::participant_configuration(
                false,  // server
                true,  // is server 1
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
                ),
            0  // domain
            ), // Client 0

        test::router_configuration(
            test::participant_configuration(
                false,  // server
                false,  // is server 1
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
                ),
            1  // domain
            ), // Client 1

        test::router_configuration(
            test::participant_configuration(
                true,  // server
                true,  // is server 1 [not used]
                participants::types::TransportProtocol::udp, // transport protocol
                participants::types::IpVersion::v4 // ip version
                ),
            66,  // domain
            false  // has simple
            ) // Repeater
        );
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

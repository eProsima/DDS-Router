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

/**
 * @file ParticipantFactoryTest.cpp
 *
 */

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_core/dynamic/DiscoveryDatabase.hpp>
#include <ddspipe_core/efficiency/payload/PayloadPool.hpp>
#include <ddspipe_core/efficiency/payload/FastPayloadPool.hpp>

#include <ddspipe_participants/configuration/EchoParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/DiscoveryServerParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/XmlParticipantConfiguration.hpp>

#include <ddspipe_core/interface/IParticipant.hpp>
#include <ddspipe_participants/participant/auxiliar/EchoParticipant.hpp>
#include <ddspipe_participants/participant/rtps/SimpleParticipant.hpp>
#include <ddspipe_participants/participant/rtps/DiscoveryServerParticipant.hpp>
#include <ddspipe_participants/participant/rtps/InitialPeersParticipant.hpp>
#include <ddspipe_participants/participant/dds/XmlParticipant.hpp>

#include <ddspipe_participants/testing/random_values.hpp>

#include <ddsrouter_core/core/ParticipantFactory.hpp>

using namespace eprosima;
using namespace eprosima::ddsrouter::core;

class EchoTestClass : public ddspipe::participants::EchoParticipant
{
public:

    using ddspipe::participants::EchoParticipant::configuration_;  // Make protected member accessible
    EchoTestClass(
            std::shared_ptr<ddspipe::participants::EchoParticipant> echo_participant)
        : ddspipe::participants::EchoParticipant(*echo_participant)
    {
    }

};

class SimpleTestClass : public ddspipe::participants::rtps::SimpleParticipant
{
public:

    using ddspipe::participants::rtps::SimpleParticipant::configuration_;  // Make protected member accessible
    SimpleTestClass(
            std::shared_ptr<ddspipe::participants::rtps::SimpleParticipant> simple_participant)
        : ddspipe::participants::rtps::SimpleParticipant(*simple_participant)
    {
    }

};

class DiscoveryServerTestClass : public ddspipe::participants::rtps::DiscoveryServerParticipant
{
public:

    using ddspipe::participants::rtps::DiscoveryServerParticipant::configuration_;  // Make protected member accessible
    DiscoveryServerTestClass(
            std::shared_ptr<ddspipe::participants::rtps::DiscoveryServerParticipant> discovery_server_participant)
        : ddspipe::participants::rtps::DiscoveryServerParticipant(*discovery_server_participant)
    {
    }

};

class InitialPeersTestClass : public ddspipe::participants::rtps::InitialPeersParticipant
{
public:

    using ddspipe::participants::rtps::InitialPeersParticipant::configuration_;  // Make protected member accessible
    InitialPeersTestClass(
            std::shared_ptr<ddspipe::participants::rtps::InitialPeersParticipant> initial_peers_participant)
        : ddspipe::participants::rtps::InitialPeersParticipant(*initial_peers_participant)
    {
    }

};

// class XMLTestClass : public ddspipe::participants::dds::XmlParticipant
// {
// public:
//     using ddspipe::participants::dds::XmlParticipant::configuration_;  // Make protected member accessible
//     XMLTestClass(std::shared_ptr<ddspipe::participants::dds::XmlParticipant> xml_participant)
//     : ddspipe::participants::dds::XmlParticipant(*xml_participant)
//     {
//     }
// };

/**
 * TODO
 */
TEST(ParticipantFactoryTest, trivial)
{
    ParticipantFactory participant_factory;
}

/**
 * TODO
 */
TEST(ParticipantFactoryTest, create_echo_participant)
{
    ParticipantFactory participant_factory;

    auto configuration = std::make_shared<ddspipe::participants::EchoParticipantConfiguration>();
    auto payload_pool = std::make_shared<ddspipe::core::FastPayloadPool>();
    auto discovery_database = std::make_shared<ddspipe::core::DiscoveryDatabase>();

    std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
        types::ParticipantKind::echo, configuration, payload_pool, discovery_database);

    ASSERT_TRUE(i_participant) << "Failed to create I Participant";

    std::shared_ptr<ddspipe::participants::EchoParticipant> echo_participant =
            std::dynamic_pointer_cast<ddspipe::participants::EchoParticipant>(i_participant);

    ASSERT_TRUE(echo_participant) << "Failed to create Echo Participant";

    EchoTestClass participant(echo_participant);

    ASSERT_EQ(participant.configuration_->app_id, "DDS_ROUTER");
    ASSERT_EQ(participant.configuration_->app_metadata, "");

}

/**
 * TODO
 */
TEST(ParticipantFactoryTest, create_simple_participant)
{
    ParticipantFactory participant_factory;

    auto configuration = std::make_shared<ddspipe::participants::SimpleParticipantConfiguration>();
    auto payload_pool = std::make_shared<ddspipe::core::FastPayloadPool>();
    auto discovery_database = std::make_shared<ddspipe::core::DiscoveryDatabase>();

    std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
        types::ParticipantKind::simple, configuration, payload_pool, discovery_database);

    std::shared_ptr<ddspipe::participants::rtps::SimpleParticipant> simple_participant =
            std::dynamic_pointer_cast<ddspipe::participants::rtps::SimpleParticipant>(i_participant);

    ASSERT_TRUE(simple_participant) << "Failed to create Simple Participant";

    SimpleTestClass participant(simple_participant);

    ASSERT_EQ(participant.configuration_->app_id, "DDS_ROUTER");
    ASSERT_EQ(participant.configuration_->app_metadata, "");
}

/**
 * TODO
 */
TEST(ParticipantFactoryTest, create_discovery_server_participant)
{
    ParticipantFactory participant_factory;

    auto configuration = std::make_shared<ddspipe::participants::DiscoveryServerParticipantConfiguration>();
    configuration->listening_addresses.insert(ddspipe::participants::testing::random_address());
    auto payload_pool = std::make_shared<ddspipe::core::FastPayloadPool>();
    auto discovery_database = std::make_shared<ddspipe::core::DiscoveryDatabase>();

    std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
        types::ParticipantKind::discovery_server, configuration, payload_pool, discovery_database);

    std::shared_ptr<ddspipe::participants::rtps::DiscoveryServerParticipant> discovery_server_participant =
            std::dynamic_pointer_cast<ddspipe::participants::rtps::DiscoveryServerParticipant>(i_participant);

    ASSERT_TRUE(discovery_server_participant) << "Failed to create Discovery Server Participant";

    DiscoveryServerTestClass participant(discovery_server_participant);

    ASSERT_EQ(participant.configuration_->app_id, "DDS_ROUTER");
    ASSERT_EQ(participant.configuration_->app_metadata, "");
}

/**
 * TODO
 */
TEST(ParticipantFactoryTest, create_initial_peers_participant)
{
    ParticipantFactory participant_factory;

    auto configuration = std::make_shared<ddspipe::participants::InitialPeersParticipantConfiguration>();
    configuration->listening_addresses.insert(ddspipe::participants::testing::random_address());
    auto payload_pool = std::make_shared<ddspipe::core::FastPayloadPool>();
    auto discovery_database = std::make_shared<ddspipe::core::DiscoveryDatabase>();

    std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
        types::ParticipantKind::initial_peers, configuration, payload_pool, discovery_database);

    std::shared_ptr<ddspipe::participants::rtps::InitialPeersParticipant> initial_peers_participant =
            std::dynamic_pointer_cast<ddspipe::participants::rtps::InitialPeersParticipant>(i_participant);

    ASSERT_TRUE(initial_peers_participant) << "Failed to create Initial Peers Participant";

    InitialPeersTestClass participant(initial_peers_participant);

    ASSERT_EQ(participant.configuration_->app_id, "DDS_ROUTER");
    ASSERT_EQ(participant.configuration_->app_metadata, "");
}

/**
 * TODO
 */
TEST(ParticipantFactoryTest, create_xml_participant)
{
    ParticipantFactory participant_factory;

    auto configuration = std::make_shared<ddspipe::participants::XmlParticipantConfiguration>();
    auto payload_pool = std::make_shared<ddspipe::core::FastPayloadPool>();
    auto discovery_database = std::make_shared<ddspipe::core::DiscoveryDatabase>();

    std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
        types::ParticipantKind::xml, configuration, payload_pool, discovery_database);

    std::shared_ptr<ddspipe::participants::dds::XmlParticipant> xml_participant =
            std::dynamic_pointer_cast<ddspipe::participants::dds::XmlParticipant>(i_participant);

    ASSERT_TRUE(xml_participant) << "Failed to create XML Participant";
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

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
#include <ddspipe_participants/participant/dds/SimpleParticipant.hpp>
#include <ddspipe_participants/participant/dds/DiscoveryServerParticipant.hpp>
#include <ddspipe_participants/participant/dds/InitialPeersParticipant.hpp>
#include <ddspipe_participants/participant/dds/XmlParticipant.hpp>

#include <ddspipe_participants/testing/random_values.hpp>

#include <ddsrouter_core/core/ParticipantFactory.hpp>

using namespace eprosima;
using namespace eprosima::ddsrouter::core;

/**
 * This class is a subclass of ddspipe::participants::EchoParticipant.
 * It provides public access to the protected member 'configuration_' from its base class
 * ddspipe::participants::EchoParticipant.
 */
class EchoTestClass : public ddspipe::participants::EchoParticipant
{
public:

    using ddspipe::participants::EchoParticipant::configuration_;  // Make protected member accessible
};

/**
 * This class is a subclass of ddspipe::participants::SimpleParticipant.
 * It provides public access to the protected member 'configuration_' from its base class
 * ddspipe::participants::SimpleParticipant.
 */
class SimpleTestClass : public ddspipe::participants::dds::SimpleParticipant
{
public:

    using ddspipe::participants::dds::SimpleParticipant::configuration_;  // Make protected member accessible
};

/**
 * This class is a subclass of ddspipe::participants::DiscoveryServerParticipant.
 * It provides public access to the protected member 'configuration_' from its base class
 * ddspipe::participants::DiscoveryServerParticipant.
 */
class DiscoveryServerTestClass : public ddspipe::participants::dds::DiscoveryServerParticipant
{
public:

    using ddspipe::participants::dds::DiscoveryServerParticipant::configuration_;  // Make protected member accessible
};

/**
 * This class is a subclass of ddspipe::participants::InitialPeersParticipant.
 * It provides public access to the protected member 'configuration_' from its base class
 * ddspipe::participants::InitialPeersParticipant.
 */
class InitialPeersTestClass : public ddspipe::participants::dds::InitialPeersParticipant
{
public:

    using ddspipe::participants::dds::InitialPeersParticipant::configuration_;  // Make protected member accessible
};

/**
 * This class is a subclass of ddspipe::participants::XmlParticipant.
 * It provides public access to the protected member 'configuration_' from its base class
 * ddspipe::participants::XmlParticipant.
 */
class XMLTestClass : public ddspipe::participants::dds::XmlParticipant
{
public:

    using ddspipe::participants::dds::XmlParticipant::configuration_;  // Make protected member accessible
};

/**
 * This test case is for the ParticipantFactory class, specifically testing the creation
 * of an EchoParticipant. The test checks whether the created EchoParticipant has the
 * expected configuration values.
 */
TEST(ParticipantFactoryTest, create_echo_participant)
{
    {
        ParticipantFactory participant_factory;

        auto configuration = std::make_shared<ddspipe::participants::EchoParticipantConfiguration>();
        std::shared_ptr<ddspipe::core::PayloadPool> payload_pool(new ddspipe::core::FastPayloadPool());
        std::shared_ptr<ddspipe::core::DiscoveryDatabase> discovery_database(new ddspipe::core::DiscoveryDatabase());

        std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
            types::ParticipantKind::echo, configuration, payload_pool, discovery_database);

        ASSERT_TRUE(i_participant) << "Failed to create I Participant";

        std::shared_ptr<EchoTestClass> echo_participant =
                std::static_pointer_cast<EchoTestClass>(i_participant);

        ASSERT_TRUE(echo_participant) << "Failed to create Echo Participant";

        ASSERT_EQ(echo_participant->configuration_->app_id, "DDS_ROUTER");
        ASSERT_EQ(echo_participant->configuration_->app_metadata, "");
    }
}

/**
 * This test case is for the ParticipantFactory class, specifically testing the creation
 * of an SimpleParticipant. The test checks whether the created SimpleParticipant has the
 * expected configuration values.
 */
TEST(ParticipantFactoryTest, create_simple_participant)
{
    {
        ParticipantFactory participant_factory;

        auto configuration = std::make_shared<ddspipe::participants::SimpleParticipantConfiguration>();
        std::shared_ptr<ddspipe::core::PayloadPool> payload_pool(new ddspipe::core::FastPayloadPool());
        std::shared_ptr<ddspipe::core::DiscoveryDatabase> discovery_database(new ddspipe::core::DiscoveryDatabase());

        std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
            types::ParticipantKind::simple, configuration, payload_pool, discovery_database);

        std::shared_ptr<SimpleTestClass> simple_participant =
                std::static_pointer_cast<SimpleTestClass>(i_participant);

        ASSERT_TRUE(simple_participant) << "Failed to create Simple Participant";

        ASSERT_EQ(simple_participant->configuration_->app_id, "DDS_ROUTER");
        ASSERT_EQ(simple_participant->configuration_->app_metadata, "");
    }
}

/**
 * This test case is for the ParticipantFactory class, specifically testing the creation
 * of an DiscoveryServerParticipant. The test checks whether the created DiscoveryServerParticipant has the
 * expected configuration values.
 */
TEST(ParticipantFactoryTest, create_discovery_server_participant)
{
    {
        ParticipantFactory participant_factory;

        auto configuration = std::make_shared<ddspipe::participants::DiscoveryServerParticipantConfiguration>();
        configuration->listening_addresses.insert(ddspipe::participants::testing::random_address());
        std::shared_ptr<ddspipe::core::PayloadPool> payload_pool(new ddspipe::core::FastPayloadPool());
        std::shared_ptr<ddspipe::core::DiscoveryDatabase> discovery_database(new ddspipe::core::DiscoveryDatabase());

        std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
            types::ParticipantKind::discovery_server, configuration, payload_pool, discovery_database);

        std::shared_ptr<DiscoveryServerTestClass> discovery_server_participant =
                std::static_pointer_cast<DiscoveryServerTestClass>(i_participant);

        ASSERT_TRUE(discovery_server_participant) << "Failed to create Discovery Server Participant";

        ASSERT_EQ(discovery_server_participant->configuration_->app_id, "DDS_ROUTER");
        ASSERT_EQ(discovery_server_participant->configuration_->app_metadata, "");
    }
}

/**
 * This test case is for the ParticipantFactory class, specifically testing the creation
 * of an InitialPeersParticipant. The test checks whether the created InitialPeersParticipant has the
 * expected configuration values.
 */
TEST(ParticipantFactoryTest, create_initial_peers_participant)
{
    {
        ParticipantFactory participant_factory;

        auto configuration = std::make_shared<ddspipe::participants::InitialPeersParticipantConfiguration>();
        configuration->listening_addresses.insert(ddspipe::participants::testing::random_address());
        std::shared_ptr<ddspipe::core::PayloadPool> payload_pool(new ddspipe::core::FastPayloadPool());
        std::shared_ptr<ddspipe::core::DiscoveryDatabase> discovery_database(new ddspipe::core::DiscoveryDatabase());

        std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
            types::ParticipantKind::initial_peers, configuration, payload_pool, discovery_database);

        std::shared_ptr<InitialPeersTestClass> initial_peers_participant =
                std::static_pointer_cast<InitialPeersTestClass>(i_participant);

        ASSERT_TRUE(initial_peers_participant) << "Failed to create Initial Peers Participant";

        ASSERT_EQ(initial_peers_participant->configuration_->app_id, "DDS_ROUTER");
        ASSERT_EQ(initial_peers_participant->configuration_->app_metadata, "");
    }
}

/**
 * This test case is for the ParticipantFactory class, specifically testing the creation
 * of an XmlParticipant. The test checks whether the created XmlParticipant has the
 * expected configuration values.
 */
TEST(ParticipantFactoryTest, create_xml_participant)
{
    {
        ParticipantFactory participant_factory;

        auto configuration = std::make_shared<ddspipe::participants::XmlParticipantConfiguration>();
        std::shared_ptr<ddspipe::core::PayloadPool> payload_pool(new ddspipe::core::FastPayloadPool());
        std::shared_ptr<ddspipe::core::DiscoveryDatabase> discovery_database(new ddspipe::core::DiscoveryDatabase());

        std::shared_ptr<eprosima::ddspipe::core::IParticipant> i_participant = participant_factory.create_participant(
            types::ParticipantKind::xml, configuration, payload_pool, discovery_database);

        std::shared_ptr<XMLTestClass> xml_participant =
                std::static_pointer_cast<XMLTestClass>(i_participant);

        ASSERT_TRUE(xml_participant) << "Failed to create XML Participant";

        ASSERT_EQ(xml_participant->configuration_->app_id, "DDS_ROUTER");
        ASSERT_EQ(xml_participant->configuration_->app_metadata, "");
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

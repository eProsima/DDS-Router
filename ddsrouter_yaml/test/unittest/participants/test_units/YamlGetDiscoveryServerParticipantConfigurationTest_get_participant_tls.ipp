// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>
#include <test_utils.hpp>

#include <ddsrouter_core/types/participant/ParticipantKind.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>
#include <ddsrouter_yaml/YamlReader.hpp>

#include "../../YamlConfigurationTestUtils.hpp"

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::yaml;

/**
 * Test get Participant Configuration from yaml specifing TLS configuration
 *
 * POSITIVE CASES:
 * - client TLS
 *
 * NEGATIVE CASES:
 * - empty tls
 * - incorrect tls format
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_tls)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::local_discovery_server);
    core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id();
    core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();

    // client TLS
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add TLS
        yml_participant[TLS_TAG] = Yaml();
        yml_participant[TLS_TAG][TLS_CA_TAG] = "ca.pem";

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::configuration::DiscoveryServerParticipantConfiguration result =
                YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_TRUE(result.tls_active());
        ASSERT_TRUE(result.tls_configuration()->can_be_client());
        ASSERT_FALSE(result.tls_configuration()->can_be_server());
    }

    // empty tls
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add empty TLS
        yml_participant[TLS_TAG] = Yaml();

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ASSERT_THROW(
            core::configuration::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }

    // incorrect tls format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add empty TLS
        Yaml yml_array;
        yml_array.push_back("ca.cert");
        yml_participant[TLS_TAG] = yml_array;

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ASSERT_THROW(
            core::configuration::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }
}

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
 * Test get DS Participant Configuration from yaml fail cases
 *
 * NEGATIVE CASES:
 * - empty
 * - no id
 * - no type
 * - no discovery server guid prefix
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_negative)
{
    core::types::ParticipantKind kind(core::types::ParticipantKind::local_discovery_server);
    core::types::ParticipantId id(eprosima::ddsrouter::test::random_participant_id());
    core::types::GuidPrefix ds_guid = eprosima::ddsrouter::test::random_guid_prefix();

    // empty
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            core::configuration::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }

    // no id
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, ds_guid);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            core::configuration::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }

    // no type
    {
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, ds_guid);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            core::configuration::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }

    // no discovery server guid prefix
    {
        Yaml yml;
        Yaml yml_participant;
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            core::configuration::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::configuration::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            utils::ConfigurationException);
    }
}

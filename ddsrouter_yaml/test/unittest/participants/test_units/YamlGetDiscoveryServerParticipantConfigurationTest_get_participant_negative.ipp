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

#include <cpp_utils/testing/gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddspipe_core/testing/random_values.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/dds/DomainId.hpp>

#include <ddspipe_yaml/YamlReader.hpp>
#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

#include <test_utils.hpp>

using namespace eprosima;

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
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::discovery_server;
    ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id();
    ddspipe::core::types::GuidPrefix ds_guid = ddspipe::core::testing::random_guid_prefix();

    // empty
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // no id
    {
        // Create structure
        Yaml yml;
        Yaml yml_participant;
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, ds_guid);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }

    // no discovery server guid prefix
    {
        Yaml yml;
        Yaml yml_participant;
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        yml["participant"] = yml_participant;

        // Read Yaml
        ASSERT_THROW(
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            eprosima::utils::ConfigurationException);
    }
}

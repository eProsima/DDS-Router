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
 * Test get Participant Configuration from yaml specifing domain
 *
 * POSITIVE CASES:
 * - specify domain
 *
 * NEGATIVE CASES:
 * - incorrect domain format
 */
TEST(YamlGetDiscoveryServerParticipantConfigurationTest, get_participant_domain)
{
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::discovery_server;
    ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id();
    ddspipe::core::types::GuidPrefix guid_prefix = ddspipe::core::testing::random_guid_prefix();

    // specify domain
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add domain
        ddspipe::core::types::DomainId domain = ddspipe::core::testing::random_domain();
        ddspipe::yaml::testing::domain_to_yaml(yml_participant, domain);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        ddspipe::participants::DiscoveryServerParticipantConfiguration result =
                ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
                        "participant",
                        ddspipe::yaml::YamlReaderVersion::LATEST);

        // Check result
        ASSERT_EQ(domain, result.domain);
    }

    // incorrect domain format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
        ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
        ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add incorrect domain
        yml_participant["domain"] = "DOMAIN";

        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            ddspipe::participants::DiscoveryServerParticipantConfiguration result =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
            "participant", ddspipe::yaml::YamlReaderVersion::LATEST),
            utils::ConfigurationException);
    }
}

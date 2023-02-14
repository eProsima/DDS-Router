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

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;

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
    core::types::ParticipantKind kind(core::types::ParticipantKind::local_discovery_server);
    core::types::ParticipantId id = eprosima::ddsrouter::test::random_participant_id();
    core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();

    // specify domain
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add domain
        core::types::DomainId domain = eprosima::ddsrouter::test::random_domain();
        yaml::test::domain_to_yaml(yml_participant, domain);

        yml["participant"] = yml_participant;

        // Get configuration object from yaml
        core::DiscoveryServerParticipantConfiguration result =
                YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml, "participant",
                        LATEST);

        // Check result
        ASSERT_EQ(domain, result.domain);
    }

    // incorrect domain format
    {
        Yaml yml;
        Yaml yml_participant;

        // Add required fields
        yaml::test::participantid_to_yaml(yml_participant, id);
        yaml::test::participantkind_to_yaml(yml_participant, kind);
        yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

        // Add incorrect domain
        yml_participant["domain"] = "DOMAIN";

        yml["participant"] = yml_participant;

        // Get configuration object from yaml and expect fail
        ASSERT_THROW(
            core::DiscoveryServerParticipantConfiguration result =
            YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml, "participant", LATEST),
            eprosima::utils::ConfigurationException);
    }
}

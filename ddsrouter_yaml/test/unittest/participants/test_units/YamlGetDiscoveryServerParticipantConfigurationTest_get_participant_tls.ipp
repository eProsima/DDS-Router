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
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <ddspipe_yaml/testing/generate_yaml.hpp>

#include <ddsrouter_yaml/testing/generate_yaml.hpp>

#include <test_utils.hpp>

/**
 * Test get specifing TLS configuration
 *
 * POSITIVE CASES:
 * - client TLS
 * - server TLS
 * - client & server TLS
 * - Configuration with inactive TLS
 *
 * NEGATIVE CASES:
 * - empty tls
 * - incorrect tls format
 */

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_client)
{
    Yaml yml;

    // Add TLS
    yml[ddspipe::yaml::TLS_TAG] = Yaml();
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_CA_TAG] = "ca.pem";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_SNI_HOST_TAG] = "my_server.com";

    ddspipe::participants::types::TlsConfiguration tls_configuration =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::types::TlsConfiguration>(yml, ddspipe::yaml::TLS_TAG,
                    ddspipe::yaml::YamlReaderVersion::LATEST);

    // Check tls_configuration
    ASSERT_TRUE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::client>());
    ASSERT_FALSE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::both>());
    ASSERT_FALSE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::server>());
    ASSERT_TRUE(tls_configuration.is_active());

    ASSERT_EQ(tls_configuration.certificate_authority_file, "ca.pem");
    ASSERT_EQ(tls_configuration.sni_server_name, "my_server.com");
}

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_server)
{
    Yaml yml;

    // Add TLS
    yml[ddspipe::yaml::TLS_TAG] = Yaml();
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_PRIVATE_KEY_TAG] = "pk-file";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_PASSWORD_TAG] = "pwd-file";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_CERT_TAG] = "cert-chain-file";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_DHPARAMS_TAG] = "dhp-file";

    ddspipe::participants::types::TlsConfiguration tls_configuration =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::types::TlsConfiguration>(yml, ddspipe::yaml::TLS_TAG,
                    ddspipe::yaml::YamlReaderVersion::LATEST);

    // Check tls_configuration
    ASSERT_FALSE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::client>());
    ASSERT_FALSE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::both>());
    ASSERT_TRUE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::server>());
    ASSERT_TRUE(tls_configuration.is_active());

    ASSERT_EQ(tls_configuration.private_key_file_password, "pwd-file");
    ASSERT_EQ(tls_configuration.private_key_file, "pk-file");
    ASSERT_EQ(tls_configuration.certificate_chain_file, "cert-chain-file");
    ASSERT_EQ(tls_configuration.dh_params_file, "dhp-file");
}


TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_client_server)
{
    Yaml yml;

    // Add TLS
    yml[ddspipe::yaml::TLS_TAG] = Yaml();
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_CA_TAG] = "ca.pem";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_PRIVATE_KEY_TAG] = "pk-file";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_PASSWORD_TAG] = "pwd-file";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_CERT_TAG] = "cert-chain-file";
    yml[ddspipe::yaml::TLS_TAG][ddspipe::yaml::TLS_DHPARAMS_TAG] = "dhp-file";

    ddspipe::participants::types::TlsConfiguration tls_configuration =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::types::TlsConfiguration>(yml, ddspipe::yaml::TLS_TAG,
                    ddspipe::yaml::YamlReaderVersion::LATEST);

    // Check tls_configuration
    ASSERT_TRUE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::client>());
    ASSERT_TRUE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::both>());
    ASSERT_TRUE(tls_configuration.compatible<ddspipe::participants::types::TlsKind::server>());
    ASSERT_TRUE(tls_configuration.is_active());

    ASSERT_EQ(tls_configuration.certificate_authority_file, "ca.pem");
    ASSERT_EQ(tls_configuration.private_key_file_password, "pwd-file");
    ASSERT_EQ(tls_configuration.private_key_file, "pk-file");
    ASSERT_EQ(tls_configuration.certificate_chain_file, "cert-chain-file");
    ASSERT_EQ(tls_configuration.dh_params_file, "dhp-file");
}


TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_inactive)
{
    // ParticipantConfiguration with inactive TLS
    ddsrouter::core::types::ParticipantKind kind = ddsrouter::core::types::ParticipantKind::discovery_server;
    ddspipe::core::types::ParticipantId id = ddspipe::core::testing::random_participant_id();
    ddspipe::core::types::GuidPrefix guid_prefix = ddspipe::core::testing::random_guid_prefix();

    Yaml yml;
    Yaml yml_participant;

    // Add required fields
    ddspipe::yaml::testing::participantid_to_yaml(yml_participant, id);
    ddsrouter::yaml::testing::participantkind_to_yaml(yml_participant, kind);
    ddspipe::yaml::testing::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

    // Add addresses (so that participant configuration is valid)
    Yaml yml_listening_addresses;
    Yaml yml_address;
    ddspipe::participants::types::Address address = ddspipe::participants::testing::random_address();
    ddspipe::yaml::testing::address_to_yaml(yml_address, address);
    yml_listening_addresses.push_back(yml_address);

    yml_participant[ddspipe::yaml::LISTENING_ADDRESSES_TAG] = yml_listening_addresses;


    // Store participant yaml node
    yml["participant"] = yml_participant;
    // No TLS tag insertion

    // Get configuration object from yaml
    auto ds_participant_cfg =
            ddspipe::yaml::YamlReader::get<ddspipe::participants::DiscoveryServerParticipantConfiguration>(yml,
                    "participant",
                    ddspipe::yaml::YamlReaderVersion::LATEST);

    // Check tls_configuration
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.compatible<ddspipe::participants::types::TlsKind::client>());
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.compatible<ddspipe::participants::types::TlsKind::both>());
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.compatible<ddspipe::participants::types::TlsKind::server>());
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.is_active());
}

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_incorrect_empty)
{
    // incorrect tls yaml format: tls tag with no content
    Yaml yml;

    // Add empty TLS
    yml[ddspipe::yaml::TLS_TAG] = Yaml();

    ASSERT_THROW(
        ddspipe::participants::types::TlsConfiguration tls_configuration = ddspipe::yaml::YamlReader::get<ddspipe::participants::types::TlsConfiguration>(
            yml, ddspipe::yaml::TLS_TAG,
            ddspipe::yaml::YamlReaderVersion::LATEST),
        eprosima::utils::ConfigurationException);
}

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_incorrect_format)
{
    // incorrect tls yaml format: tls tag with with array-like contents
    Yaml yml;

    // Add empty TLS
    Yaml yml_array;
    yml_array.push_back("ca.cert");
    yml[ddspipe::yaml::TLS_TAG] = yml_array;

    // Get configuration object from yaml
    ASSERT_THROW(
        ddspipe::participants::types::TlsConfiguration tls_configuration = ddspipe::yaml::YamlReader::get<ddspipe::participants::types::TlsConfiguration>(
            yml, ddspipe::yaml::TLS_TAG,
            ddspipe::yaml::YamlReaderVersion::LATEST),
        eprosima::utils::ConfigurationException);
}

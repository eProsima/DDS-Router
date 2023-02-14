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

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddspipe::yaml;

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
    yml[TLS_TAG] = Yaml();
    yml[TLS_TAG][TLS_CA_TAG] = "ca.pem";
    yml[TLS_TAG][TLS_SNI_HOST_TAG] = "my_server.com";

    core::types::security::TlsConfiguration tls_configuration =
            YamlReader::get<core::types::security::TlsConfiguration>(yml, TLS_TAG, LATEST);

    // Check tls_configuration
    ASSERT_TRUE(tls_configuration.compatible<core::types::security::TlsKind::client>());
    ASSERT_FALSE(tls_configuration.compatible<core::types::security::TlsKind::both>());
    ASSERT_FALSE(tls_configuration.compatible<core::types::security::TlsKind::server>());
    ASSERT_TRUE(tls_configuration.is_active());

    ASSERT_EQ(tls_configuration.certificate_authority_file, "ca.pem");
    ASSERT_EQ(tls_configuration.sni_server_name, "my_server.com");
}

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_server)
{
    Yaml yml;

    // Add TLS
    yml[TLS_TAG] = Yaml();
    yml[TLS_TAG][TLS_PRIVATE_KEY_TAG] = "pk-file";
    yml[TLS_TAG][TLS_PASSWORD_TAG] = "pwd-file";
    yml[TLS_TAG][TLS_CERT_TAG] = "cert-chain-file";
    yml[TLS_TAG][TLS_DHPARAMS_TAG] = "dhp-file";

    core::types::security::TlsConfiguration tls_configuration =
            YamlReader::get<core::types::security::TlsConfiguration>(yml, TLS_TAG, LATEST);

    // Check tls_configuration
    ASSERT_FALSE(tls_configuration.compatible<core::types::security::TlsKind::client>());
    ASSERT_FALSE(tls_configuration.compatible<core::types::security::TlsKind::both>());
    ASSERT_TRUE(tls_configuration.compatible<core::types::security::TlsKind::server>());
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
    yml[TLS_TAG] = Yaml();
    yml[TLS_TAG][TLS_CA_TAG] = "ca.pem";
    yml[TLS_TAG][TLS_PRIVATE_KEY_TAG] = "pk-file";
    yml[TLS_TAG][TLS_PASSWORD_TAG] = "pwd-file";
    yml[TLS_TAG][TLS_CERT_TAG] = "cert-chain-file";
    yml[TLS_TAG][TLS_DHPARAMS_TAG] = "dhp-file";

    core::types::security::TlsConfiguration tls_configuration =
            YamlReader::get<core::types::security::TlsConfiguration>(yml, TLS_TAG, LATEST);

    // Check tls_configuration
    ASSERT_TRUE(tls_configuration.compatible<core::types::security::TlsKind::client>());
    ASSERT_TRUE(tls_configuration.compatible<core::types::security::TlsKind::both>());
    ASSERT_TRUE(tls_configuration.compatible<core::types::security::TlsKind::server>());
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
    core::types::ParticipantId id(eprosima::ddsrouter::test::random_participant_id());
    core::types::ParticipantKind kind(core::types::ParticipantKind::local_discovery_server);
    core::types::GuidPrefix guid_prefix = eprosima::ddsrouter::test::random_guid_prefix();

    Yaml yml;
    Yaml yml_participant;

    // Add required fields
    yaml::test::participantid_to_yaml(yml_participant, id);
    yaml::test::participantkind_to_yaml(yml_participant, kind);
    yaml::test::discovery_server_guid_prefix_to_yaml(yml_participant, guid_prefix);

    // Add addresses (so that participant configuration is valid)
    Yaml yml_listening_addresses;
    Yaml yml_address;
    participants::types::Address address = eprosima::ddsrouter::test::random_address();
    yaml::test::address_to_yaml(yml_address, address);
    yml_listening_addresses.push_back(yml_address);

    yml_participant[LISTENING_ADDRESSES_TAG] = yml_listening_addresses;


    // Store participant yaml node
    yml["participant"] = yml_participant;
    // No TLS tag insertion

    // Get configuration object from yaml
    auto ds_participant_cfg = YamlReader::get<core::DiscoveryServerParticipantConfiguration>(yml,
                    "participant",
                    LATEST);

    // Check tls_configuration
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.compatible<core::types::security::TlsKind::client>());
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.compatible<core::types::security::TlsKind::both>());
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.compatible<core::types::security::TlsKind::server>());
    ASSERT_FALSE(ds_participant_cfg.tls_configuration.is_active());
}

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_incorrect_empty)
{
    // incorrect tls yaml format: tls tag with no content
    Yaml yml;

    // Add empty TLS
    yml[TLS_TAG] = Yaml();

    ASSERT_THROW(
        core::types::security::TlsConfiguration tls_configuration = YamlReader::get<core::types::security::TlsConfiguration>(
            yml, TLS_TAG,
            LATEST),
        eprosima::utils::ConfigurationException);
}

TEST(YamlGetDiscoveryServerParticipantConfigurationTest, tls_configuration_incorrect_format)
{
    // incorrect tls yaml format: tls tag with with array-like contents
    Yaml yml;

    // Add empty TLS
    Yaml yml_array;
    yml_array.push_back("ca.cert");
    yml[TLS_TAG] = yml_array;

    // Get configuration object from yaml
    ASSERT_THROW(
        core::types::security::TlsConfiguration tls_configuration = YamlReader::get<core::types::security::TlsConfiguration>(
            yml, TLS_TAG,
            LATEST),
        eprosima::utils::ConfigurationException);
}

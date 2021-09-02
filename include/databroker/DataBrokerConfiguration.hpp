// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DataBrokerConfiguration.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_DATABROKERCONFIGURATION_HPP
#define EPROSIMA_DATABROKER_DATABROKERCONFIGURATION_HPP

#include <string>
#include <vector>

#include <fastdds/rtps/common/GuidPrefix_t.hpp>

#include <databroker/Address.hpp>

namespace eprosima {
namespace databroker {

#define DEFAULT_CONFIGURATION_FILE "DATABROKER_CONFIGURATION.yaml"

constexpr const char* DEFAULT_PRIVATE_KEY_FILE = "databroker.key";
constexpr const char* DEFAULT_DH_PARAMS_FILE = "dh_params.pem";
constexpr const char* DEFAULT_CA_CERTIFICATE_FILE = "ca.crt";
constexpr const char* DEFAULT_CERTIFICATE_FILE = "databroker.crt";

struct DataBrokerParticipantConfiguration
{
    uint32_t domain;
};

struct DataBrokerLocalParticipantConfiguration : public DataBrokerParticipantConfiguration
{
    bool ros;
};

struct DataBrokerWANParticipantConfiguration : public DataBrokerParticipantConfiguration
{
    eprosima::fastrtps::rtps::GuidPrefix_t server_guid;
    std::vector<Address> listening_addresses;
    std::vector<Address> connection_addresses;
    bool udp;
    bool tls;
    std::string tls_private_key;
    std::string tls_password;
    std::string tls_dh_params;
    std::string tls_ca_cert;
    std::string tls_cert;
};

struct DataBrokerConfiguration
{
    DataBrokerLocalParticipantConfiguration local_configuration;
    DataBrokerWANParticipantConfiguration wan_configuration;

    std::vector<std::string> active_topics;
    uint32_t seconds;
    bool interactive;
    std::string config_file;

    static bool load_default_configuration(
            DataBrokerConfiguration& configuration);

    static bool load_configuration_file(
            DataBrokerConfiguration& configuration,
            const std::string& file_path = DEFAULT_CONFIGURATION_FILE,
            bool verbose = false);

    static bool reload_configuration_file(
            DataBrokerConfiguration& configuration,
            const std::string& file_path = "");
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKERCONFIGURATION_HPP */

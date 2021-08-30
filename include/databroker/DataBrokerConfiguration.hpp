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

struct DataBrokerConfiguration
{
    eprosima::fastrtps::rtps::GuidPrefix_t server_guid;
    std::vector<Address> listening_addresses;
    std::vector<Address> connection_addresses;
    std::vector<std::string> active_topics;
    uint32_t seconds;
    bool interactive;
    uint32_t domain;
    bool ros;
    bool udp;

    static bool load_default_configuration(
            DataBrokerConfiguration& configuration);

    static bool load_configuration_file(
            DataBrokerConfiguration& configuration,
            const std::string& file_path = "DATABROKER_CONFIGURATION.yaml",
            bool verbose = false);
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKERCONFIGURATION_HPP */

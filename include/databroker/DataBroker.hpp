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
 * @file DataBroker.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_DATABROKER_HPP
#define EPROSIMA_DATABROKER_DATABROKER_HPP

#include <string>
#include <vector>

#include <databroker/DataBrokerListener.hpp>
#include <databroker/DataBrokerParticipant.hpp>
#include <databroker/DataBrokerROSParticipant.hpp>
#include <databroker/Address.hpp>

namespace eprosima {
namespace databroker {

class DataBroker
{
public:

    DataBroker(
        const uint32_t domain,
        const eprosima::fastrtps::rtps::GuidPrefix_t& server_guid,
        const std::vector<Address>& listening_address,
        const std::vector<Address>& connection_addresses,
        bool internal_ros,
        bool udp);

    virtual ~DataBroker();

    bool init(const std::vector<std::string>& initial_topics);

    // 0 seconds means forever
    bool run(bool interactive, const uint32_t seconds = 0);

protected:

    eprosima::fastdds::dds::DomainParticipantQos default_participant_qos();

    eprosima::fastdds::dds::DomainParticipantQos wan_participant_qos();

    void add_topic_(const std::string& topic);

    void remove_topic_(const std::string& topic);

    bool run_interactive();
    bool run_time(const uint32_t seconds);

private:

    DataBrokerParticipant* local_;
    DataBrokerParticipant* wan_;

    DataBrokerListener listener_;

    std::map<std::string, bool> topics_;

    eprosima::fastrtps::rtps::GuidPrefix_t server_guid_;
    std::vector<Address> listening_addresses_;
    std::vector<Address> connection_addresses_;
    bool udp_;
    bool enabled_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKER_HPP */

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
 * @file DataBrokerParticipant.hpp
 *
 */

#ifndef EPROSIMA_DATABROKER_DATABROKERLOCALPARTICIPANT_HPP
#define EPROSIMA_DATABROKER_DATABROKERLOCALPARTICIPANT_HPP

#include <mutex>

#include <databroker/DataBrokerParticipant.hpp>

#include <StdString/StdString.h>
#include <databroker/DataBrokerConfiguration.hpp>

namespace eprosima {
namespace databroker {

class DataBrokerLocalParticipant : public DataBrokerParticipant
{
public:

    DataBrokerLocalParticipant(
            eprosima::fastdds::dds::DomainParticipantListener* listener,
            DataBrokerLocalParticipantConfiguration configuration);

    std::string name() override;

protected:

    eprosima::fastdds::dds::DataWriterQos datawriter_qos() override;

    eprosima::fastdds::dds::DataReaderQos datareader_qos() override;

    std::string type_name_() override;

    std::string topic_mangled_(
            const std::string& topic_name) override;

    const DataBrokerParticipantConfiguration& get_configuration_() override;

    DataBrokerLocalParticipantConfiguration configuration_;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* EPROSIMA_DATABROKER_DATABROKERLOCALPARTICIPANT_HPP */

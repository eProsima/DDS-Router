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
 * @file DataBrokerROSParticipant.cpp
 *
 */

#include <databroker/DataBrokerROSParticipant.hpp>

namespace eprosima {
namespace databroker {

std::string DataBrokerROSParticipant::type_name_()
{
    return "std_msgs::msg::dds_::String_";
}

std::string DataBrokerROSParticipant::topic_mangled_(
        const std::string& topic_name)
{
    return "rt/" + topic_name;
}

} /* namespace databroker */
} /* namespace eprosima */

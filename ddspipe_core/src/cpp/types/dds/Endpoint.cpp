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

#include <ddspipe_core/types/dds/Endpoint.hpp>
#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

TopicQoS Endpoint::topic_qos() const noexcept
{
    return topic.topic_qos;
}

bool Endpoint::is_writer() const noexcept
{
    return kind == EndpointKind::writer;
}

bool Endpoint::is_reader() const noexcept
{
    return kind == EndpointKind::reader;
}

bool Endpoint::is_server_endpoint() const noexcept
{
    return (is_reader() && RpcTopic::is_request_topic(topic)) || (is_writer() && RpcTopic::is_reply_topic(topic));
}

bool Endpoint::operator ==(
        const Endpoint& other) const noexcept
{
    return guid == other.guid;
}

std::ostream& operator <<(
        std::ostream& os,
        const Endpoint& endpoint)
{
    std::string active_str = endpoint.active ? "Active" : "Inactive";

    os <<
        "Endpoint{" << endpoint.guid <<
        ";" << endpoint.kind <<
        ";" << endpoint.topic <<
        ";" << endpoint.specific_qos <<
        ";" << active_str <<
        ";" << endpoint.discoverer_participant_id <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

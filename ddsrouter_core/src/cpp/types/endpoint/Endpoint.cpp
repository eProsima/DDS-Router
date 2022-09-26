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
 * @file Endpoint.cpp
 *
 */

#include <ddsrouter_core/types/endpoint/Endpoint.hpp>
#include <ddsrouter_utils/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

Endpoint::Endpoint() noexcept
    : kind_(EndpointKind::invalid)
{
}

Endpoint::Endpoint(
        const EndpointKind& kind,
        const Guid& guid,
        const DdsTopic& topic,
        const ParticipantId& discoverer_participant_id,
        const SpecificWriterQoS& specific_qos) noexcept
    : kind_(kind)
    , guid_(guid)
    , topic_(topic)
    , active_(true)
    , discoverer_participant_id_(discoverer_participant_id)
    , specific_qos_(specific_qos)
{
}

EndpointKind Endpoint::kind() const noexcept
{
    return kind_;
}

void Endpoint::kind(const EndpointKind& kind) noexcept
{
    kind_ = kind;
}

Guid Endpoint::guid() const noexcept
{
    return guid_;
}

TopicQoS Endpoint::topic_qos() const noexcept
{
    return topic_.topic_qos;
}

SpecificWriterQoS Endpoint::specific_qos() const noexcept
{
    return specific_qos_;
}

void Endpoint::specific_qos(const SpecificWriterQoS& specific_qos) noexcept
{
    specific_qos_ = specific_qos;
}

DdsTopic Endpoint::topic() const noexcept
{
    return topic_;
}

ParticipantId Endpoint::discoverer_participant_id() const noexcept
{
    return discoverer_participant_id_;
}

bool Endpoint::active() const noexcept
{
    return active_;
}

void Endpoint::active(
        bool status) noexcept
{
    active_ = status;
}

bool Endpoint::is_valid() const noexcept
{
    return kind_ != EndpointKind::invalid;
}

bool Endpoint::is_writer() const noexcept
{
    return kind() == EndpointKind::writer;
}

bool Endpoint::is_reader() const noexcept
{
    return kind() == EndpointKind::reader;
}

bool Endpoint::is_server_endpoint() const noexcept
{
    return (is_reader() && RPCTopic::is_request_topic(topic_)) || (is_writer() && RPCTopic::is_reply_topic(topic_));
}

Endpoint& Endpoint::operator =(
        const Endpoint& other) noexcept
{
    this->guid_ = other.guid_;
    this->active_ = other.active_;
    this->kind_ = other.kind_;
    this->topic_ = other.topic_;
    this->discoverer_participant_id_ = other.discoverer_participant_id_;
    return *this;
}

bool Endpoint::operator ==(
        const Endpoint& other) const noexcept
{
    // TODO: compare discoverer_participant_id_?
    return guid_ == other.guid() && kind_ == other.kind() && topic_ == other.topic();
}

std::ostream& operator <<(
        std::ostream& os,
        const Endpoint& endpoint)
{
    std::string active_str = endpoint.active_ ? "Active" : "Inactive";

    os <<
        "Endpoint{" << endpoint.guid_ <<
        ";" << endpoint.kind_ <<
        ";" << endpoint.topic_ <<
        ";" << endpoint.specific_qos_ <<
        ";" << active_str <<
        ";" << endpoint.discoverer_participant_id_ <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

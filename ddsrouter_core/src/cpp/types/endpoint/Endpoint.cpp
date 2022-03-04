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
    : kind_(EndpointKind::ENDPOINT_KIND_INVALID)
{
}

Endpoint::Endpoint(
        const EndpointKind& kind,
        const Guid& guid,
        const QoS& qos,
        const RealTopic& topic) noexcept
    : kind_(kind)
    , guid_(guid)
    , qos_(qos)
    , topic_(topic)
    , active_(true)
{
}

EndpointKind Endpoint::kind() const noexcept
{
    return kind_;
}

Guid Endpoint::guid() const noexcept
{
    return guid_;
}

QoS Endpoint::qos() const noexcept
{
    return qos_;
}

RealTopic Endpoint::topic() const noexcept
{
    return topic_;
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
    return kind_ != EndpointKind::ENDPOINT_KIND_INVALID;
}

bool Endpoint::is_writer() const noexcept
{
    return kind() == EndpointKind::WRITER;
}

bool Endpoint::is_reader() const noexcept
{
    return kind() == EndpointKind::READER;
}

Endpoint& Endpoint::operator =(
        const Endpoint& other) noexcept
{
    this->guid_ = other.guid_;
    this->active_ = other.active_;
    this->kind_ = other.kind_;
    this->qos_ = other.qos_;
    this->topic_ = other.topic_;
    return *this;
}

bool Endpoint::operator ==(
        const Endpoint& other) const noexcept
{
    return guid_ == other.guid() && kind_ == other.kind() && qos_ == other.qos() && topic_ == other.topic();
}

std::ostream& operator <<(
        std::ostream& os,
        const EndpointKind& kind)
{
    std::string kind_str;
    switch (kind)
    {
        case EndpointKind::ENDPOINT_KIND_INVALID:
            kind_str = "ENDPOINT_KIND_INVALID";
            break;

        case EndpointKind::WRITER:
            kind_str = "Writer";
            break;

        case EndpointKind::READER:
            kind_str = "Reader";
            break;

        default:
            utils::tsnh(utils::Formatter() << "Invalid Endpoint Kind.");
            break;
    }

    return os << kind_str;
}

std::ostream& operator <<(
        std::ostream& os,
        const Endpoint& endpoint)
{
    std::string active_str = endpoint.active_ ? "Active" : "Inactive";

    os << "Endpoint{" << endpoint.guid_ << ";" << endpoint.topic_ << ";" << endpoint.qos_ << ";" <<
        endpoint.kind_ << ";" << active_str << "}";
    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

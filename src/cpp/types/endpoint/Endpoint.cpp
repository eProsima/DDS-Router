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

#include <databroker/types/endpoint/Endpoint.hpp>

namespace eprosima {
namespace databroker {

Endpoint::Endpoint(
        const EndpointKind& kind,
        const Guid& guid,
        const QoS& qos,
        const RealTopic& topic)
    : kind_(kind)
    , guid_(guid)
    , qos_(qos)
    , topic_(topic)
    , active_(true)
{
}

EndpointKind Endpoint::kind() const
{
    return kind_;
}

Guid Endpoint::guid() const
{
    return guid_;
}

QoS Endpoint::qos() const
{
    return qos_;
}

RealTopic Endpoint::topic() const
{
    return topic_;
}

bool Endpoint::active() const
{
    return active_;
}

void Endpoint::active(
        bool status)
{
    active_ = status;
}

bool Endpoint::is_writer() const
{
    return kind() == EndpointKind::WRITER;
}

bool Endpoint::is_reader() const
{
    return kind() == EndpointKind::READER;
}

} /* namespace databroker */
} /* namespace eprosima */

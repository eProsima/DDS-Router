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
 * @file Endpoint.hpp
 */

#ifndef _DATABROKER_TYPES_ENDPOINT_HPP_
#define _DATABROKER_TYPES_ENDPOINT_HPP_

#include <databroker/types/Guid.hpp>
#include <databroker/types/QoS.hpp>
#include <databroker/topic/DatabrokerTopic.hpp>

namespace eprosima {
namespace databroker {

//! Possible kinds of the endpoint
enum EndpointKind
{
    WRITER,
    READER
};

//! Data collection to describe an Endpoint
struct Endpoint
{
    //! Kind of the endpoint
    EndpointKind kind;

    //! Unique id of the endpoint
    Guid guid;

    //! Attributes of the endpoint
    QoS qos;

    //! Topic that this endpoint belongs to
    RealTopic topic;

    //! Whether the endpoint is currently active
    bool active;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TYPES_ENDPOINT_HPP_ */

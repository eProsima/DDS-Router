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
// limitations under the License\.

#pragma once

#include <cpp_utils/macros/custom_enumeration.hpp>

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/types/dds/Guid.hpp>
#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/rpc/RpcTopic.hpp>
#include <ddspipe_core/types/topic/dds/DistributedTopic.hpp>
#include <ddspipe_core/types/dds/SpecificEndpointQoS.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

using EndpointKindType = unsigned int;

//! Possible kinds of the endpoint
ENUMERATION_BUILDER(
    EndpointKind,
    invalid,
    writer,
    reader
    );

/**
 * Data collection to describe an Endpoint
 *
 * This class works as a data storage struct with the information of a discovered Endpoint
 */
struct Endpoint
{

    //! Default Endpoint that returns an invalid one
    DDSPIPE_CORE_DllAPI Endpoint() = default;

    /********************
    * SPECIFIC GETTERS *
    ********************/

    //! Qos of the topic
    DDSPIPE_CORE_DllAPI TopicQoS topic_qos() const noexcept;

    //! Whether the endpoint is a writer
    DDSPIPE_CORE_DllAPI bool is_writer() const noexcept;

    //! Whether the endpoint is a reader
    DDSPIPE_CORE_DllAPI bool is_reader() const noexcept;

    //! Whether the endpoint belongs to a RPC server (i.e. is request reader or reply writer)
    DDSPIPE_CORE_DllAPI bool is_server_endpoint() const noexcept;

    //! Equality operator (does not take \c active and \c discoverer_participant_id into consideration)
    DDSPIPE_CORE_DllAPI bool operator ==(
            const Endpoint& other) const noexcept;

    //! Kind of the endpoint
    EndpointKind kind {EndpointKind::invalid};

    //! Unique id of the endpoint
    Guid guid {};

    //! Topic that this endpoint belongs to
    DdsTopic topic {};

    //! Specific QoS of the entity
    SpecificEndpointQoS specific_qos {};

    //! Whether the endpoint is currently active
    bool active {true};

    //! Id of participant who discovered this endpoint
    ParticipantId discoverer_participant_id {};
};

/**
 * @brief \c Endpoint to stream serialization
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Endpoint& endpoint);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

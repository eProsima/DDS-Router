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

#ifndef _DDSROUTERCORE_TYPES_ENDPOINT_ENDPOINT_HPP_
#define _DDSROUTERCORE_TYPES_ENDPOINT_ENDPOINT_HPP_

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_core/types/endpoint/QoS.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! Possible kinds of the endpoint
enum EndpointKind
{
    ENDPOINT_KIND_INVALID,
    WRITER,
    READER
};

/**
 * Data collection to describe an Endpoint
 *
 * This class works as a data storage struct with the information of a discovered Endpoint
 */
class Endpoint
{
public:

    //! Default Endpoint that returns an invalid one
    DDSROUTER_CORE_DllAPI Endpoint() noexcept;

    /**
     * Constructor with Endpoint information
     */
    DDSROUTER_CORE_DllAPI Endpoint(
            const EndpointKind& kind,
            const Guid& guid,
            const QoS& qos,
            const RealTopic& topic) noexcept;

    //! Endpoint kind getter
    DDSROUTER_CORE_DllAPI EndpointKind kind() const noexcept;

    //! Guid getter
    DDSROUTER_CORE_DllAPI Guid guid() const noexcept;

    //! QoS getter
    DDSROUTER_CORE_DllAPI QoS qos() const noexcept;

    //! Topic getter
    DDSROUTER_CORE_DllAPI RealTopic topic() const noexcept;

    //! Whether the endpoint referenced is currently active
    DDSROUTER_CORE_DllAPI bool active() const noexcept;

    //! Set active status of the Endpoint
    DDSROUTER_CORE_DllAPI void active(
            bool status) noexcept;

    //! Whether the endpoint referenced is valid
    DDSROUTER_CORE_DllAPI bool is_valid() const noexcept;

    /********************
    * SPECIFIC GETTERS *
    ********************/

    //! Whether the endpoint is a writer
    DDSROUTER_CORE_DllAPI bool is_writer() const noexcept;

    //! Whether the endpoint is a reader
    DDSROUTER_CORE_DllAPI bool is_reader() const noexcept;

    //! Copy operator
    DDSROUTER_CORE_DllAPI Endpoint& operator =(
            const Endpoint& other) noexcept;

    //! Equality operator (does not take \c active_ into consideration)
    DDSROUTER_CORE_DllAPI bool operator ==(
            const Endpoint& other) const noexcept;

protected:

    //! Kind of the endpoint
    EndpointKind kind_;

    //! Unique id of the endpoint
    Guid guid_;

    //! Attributes of the endpoint
    QoS qos_;

    //! Topic that this endpoint belongs to
    RealTopic topic_;

    //! Whether the endpoint is currently active
    bool active_;

    // Allow operator << to use private variables
    DDSROUTER_CORE_DllAPI friend std::ostream& operator <<(
            std::ostream&,
            const Endpoint&);
};

/**
 * @brief \c EndpointKind to stream serialization
 */
std::ostream& operator <<(
        std::ostream& os,
        const EndpointKind& kind);

/**
 * @brief \c Endpoint to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Endpoint& endpoint);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_ENDPOINT_ENDPOINT_HPP_ */

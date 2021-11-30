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

#ifndef _DDSROUTER_TYPES_ENDPOINT_HPP_
#define _DDSROUTER_TYPES_ENDPOINT_HPP_

#include <ddsrouter/types/endpoint/Guid.hpp>
#include <ddsrouter/types/endpoint/QoS.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {

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
    Endpoint() noexcept;

    /**
     * Constructor with Endpoint information
     */
    Endpoint(
            const EndpointKind& kind,
            const Guid& guid,
            const QoS& qos,
            const RealTopic& topic) noexcept;

    //! Endpoint kind getter
    EndpointKind kind() const noexcept;

    //! Guid getter
    Guid guid() const noexcept;

    //! QoS getter
    QoS qos() const noexcept;

    //! Topic getter
    RealTopic topic() const noexcept;

    //! Whether the endpoint referenced is currently active
    bool active() const noexcept;

    //! Set active status of the Endpoint
    void active(
            bool status) noexcept;

    bool is_valid() const noexcept;

    /********************
    * SPECIFIC GETTERS *
    ********************/

    //! Whether the endpoint is a writer
    bool is_writer() const noexcept;

    //! Whether the endpoint is a reader
    bool is_reader() const noexcept;

    //! Copy operator
    Endpoint& operator =(
            const Endpoint& other) noexcept;

    //! Equality operator
    bool operator ==(
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
    friend std::ostream& operator <<(
            std::ostream&,
            const Endpoint&);
};

/**
 * @brief \c Endpoint to stream serialization
 */
std::ostream& operator <<(
        std::ostream& os,
        const Endpoint& endpoint);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_ENDPOINT_HPP_ */

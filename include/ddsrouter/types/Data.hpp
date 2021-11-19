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
 * @file Data.hpp
 */

#ifndef _DDSROUTER_TYPES_DATA_HPP_
#define _DDSROUTER_TYPES_DATA_HPP_

#include <fastdds/rtps/common/SerializedPayload.h>

#include <ddsrouter/types/endpoint/Guid.hpp>

namespace eprosima {
namespace ddsrouter {

//! Payload references the raw data received.
using Payload = fastrtps::rtps::SerializedPayload_t;

//! Structure of the Data received from a Reader containing the data itself and the attributes of the source
struct DataReceived
{
    //! Payload of the data received. The data in this payload must belong to the PayloadPool.
    Payload payload;

    //! Guid of the source entity that has transmit the data
    Guid source_guid;
};

//! \c octet to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const eprosima::fastrtps::rtps::octet& octet);

//! \c SerializedPayload_t to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const Payload& payload);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_DATA_HPP_ */

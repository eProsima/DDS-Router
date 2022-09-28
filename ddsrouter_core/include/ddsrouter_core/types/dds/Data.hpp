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

#ifndef _DDSROUTERCORE_TYPES_DDS_DATA_HPP_
#define _DDSROUTERCORE_TYPES_DDS_DATA_HPP_

#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastdds/rtps/common/SequenceNumber.h>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/DataProperties.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

//! Kind of every unit that creates a Payload
using PayloadUnit = eprosima::fastrtps::rtps::octet;

//! Payload references the raw data received.
using Payload = eprosima::fastrtps::rtps::SerializedPayload_t;

/**
 * @brief Structure of the Data received from a Reader containing the data itself and its properties.
 *
 * Properties are related information regarding the data and QoS of the source.
 */
struct DataReceived
{
    /**
     * @brief Destroy the Data Received object
     *
     * @note Default destructor. Force \c DataReceived to be polymorphic. Implemented here to avoid creating a .cpp .
     */
    virtual ~DataReceived()
    {}

    //! Payload of the data received. The data in this payload must belong to the PayloadPool.
    Payload payload;

    //! Specific QoS and attributes of the data received
    DataProperties properties;

    /**
     * @brief Sequence Number with which the internal writer (ddsrouter writer) has sent this message
     *
     * @warning This is not the sequence number of the data received. It is the one set by writer when sending it.
     *
     * @todo This could be removed in the future if ServiceRegistry is internal to a specific class and not in
     * RPCBridge.
     */
    eprosima::fastrtps::rtps::SequenceNumber_t sent_sequence_number;
};

//! \c octet to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const PayloadUnit& octet);

//! \c SerializedPayload_t to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Payload& payload);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_DATA_HPP_ */

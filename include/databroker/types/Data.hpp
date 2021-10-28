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

#ifndef _DATABROKER_TYPES_DATA_HPP_
#define _DATABROKER_TYPES_DATA_HPP_

#include <databroker/types/QoS.hpp>

namespace eprosima {
namespace databroker {

//! Payload references the raw data received.
using Payload = void*; // maybe SerializedPayload_t

//! Structure of the Data received from a Reader containing the data itself and the attributes of the suorce
struct DataReceived
{
    //! Payload of the data received
    Payload data;

    //! Attributes of the source entity that has transmit the data
    QoS source_qos;
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TYPES_DATA_HPP_ */

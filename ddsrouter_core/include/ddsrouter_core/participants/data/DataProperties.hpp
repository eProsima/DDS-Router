// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DataProperties.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_ENDPOINT_DataProperties_HPP_
#define _DDSROUTERCORE_TYPES_ENDPOINT_DataProperties_HPP_

#include <cpp_utils/types/Fuzzy.hpp>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_core/types/dds/SpecificEndpointQoS.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

struct DDSROUTER_CORE_DllAPI RtpsDataProperties
{
    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Default DataProperties with reader less restrictive parameters
    DataProperties() = default;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    //! Minor comparison operator
    bool operator < (
            const DataProperties& other) const noexcept;

    //! Equality operator
    bool operator == (
            const DataProperties& other) const noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Specific Writer QoS of the Data
    SpecificEndpointQoS writer_qos{};

    //! Instance of the message (default no instance)
    InstanceHandle instanceHandle{};

    //! Kind of the change
    ChangeKind kind{};

    //! Source time stamp of the message
    DataTime source_timestamp{};

    //! Guid of the source entity that has transmit the data
    Guid source_guid{};

    //! Id of the participant from which the Reader has received the data.
    ParticipantId participant_receiver{};

    //! Write params associated to the received cache change
    utils::Fuzzy<eprosima::fastrtps::rtps::WriteParams> write_params{};

    //! Sequence number of the received cache change
    eprosima::fastrtps::rtps::SequenceNumber_t origin_sequence_number{};
};

/**
 * @brief \c DataProperties to stream serialization
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const DataProperties& qos);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_ENDPOINT_DataProperties_HPP_ */

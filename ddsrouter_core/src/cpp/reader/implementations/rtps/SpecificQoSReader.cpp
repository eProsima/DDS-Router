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
 * @file SpecificQoSReader.cpp
 */

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <reader/implementations/rtps/SpecificQoSReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

SpecificQoSReader::SpecificQoSReader(
        const ParticipantId& participant_id,
        const DdsTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : CommonReader(
        participant_id, topic, payload_pool, rtps_participant,
        history_attributes_(topic),
        reader_attributes_(topic),
        topic_attributes_(topic),
        reader_qos_(topic))
    , discovery_database_(discovery_database)
{
}

types::SpecificEndpointQoS SpecificQoSReader::specific_qos_of_writer_(const types::Guid& guid) const
{
    return discovery_database_->get_endpoint(guid).specific_qos();
}

void SpecificQoSReader::fill_received_data_(
        fastrtps::rtps::CacheChange_t* received_change,
        std::unique_ptr<types::DataReceived>& data_to_fill) const noexcept
{
    CommonReader::fill_received_data_(received_change, data_to_fill);

    // Find qos of writer
    try
    {
        data_to_fill->properties.writer_qos = specific_qos_of_writer_(data_to_fill->properties.source_guid);
        logDebug(
            DDSROUTER_SpecificQoSReader,
            "Set QoS " << data_to_fill->properties << " for data from " << data_to_fill->properties.source_guid << ".");
    }
    catch(const utils::InconsistencyException& e)
    {
        // Get a message from a writer not in database, this is an error.
        // Remove data and make as it has not been received.
        logError(
            DDSROUTER_SpecificQoSReader,
            "Received a message from Writer " << data_to_fill->properties.source_guid << " that is not stored in DB.");
    }
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

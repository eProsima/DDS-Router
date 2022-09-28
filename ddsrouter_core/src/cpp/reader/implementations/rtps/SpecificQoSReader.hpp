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
 * @file SpecificQoSReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_SpecificQoSReader_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_SpecificQoSReader_HPP_

#include <reader/implementations/rtps/CommonReader.hpp>
#include <dynamic/DiscoveryDatabase.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * RTPS Reader with specific QoS implements abstract CommonReader.
 *
 * This class fills the data receive information with the QoS of the Writer that has sent the data.
 * In order to access this QoS it has a reference to the DiscoveryDatabase.
 */
class SpecificQoSReader : public CommonReader
{
public:

    /**
     * @brief Construct a new SpecificQoSReader object
     *
     * Get the Attributes and TopicQoS and create the SpecificQoSReader History and the RTPS SpecificQoSReader.
     *
     * @param participant_id    Router Id of the Participant that has created this SpecificQoSReader.
     * @param topic             Topic that this SpecificQoSReader subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    SpecificQoSReader(
            const types::ParticipantId& participant_id,
            const types::DdsTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

protected:

    /**
     * @brief Get the QoS from a Writer from the \c DiscoveryDatabase .
     */
    types::SpecificEndpointQoS specific_qos_of_writer_(const types::Guid& guid) const;

    /**
     * Specializes \c CommonReader method and set the QoS of the data received.
     */
    virtual void fill_received_data_(
        fastrtps::rtps::CacheChange_t* received_change,
        std::unique_ptr<types::DataReceived>& data_to_fill) const noexcept override;

    //! Reference to the \c DiscoveryDatabase .
    std::shared_ptr<DiscoveryDatabase> discovery_database_;

};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_SpecificQoSReader_HPP_ */

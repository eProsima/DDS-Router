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
 * @file PartitionsReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_PARTITIONSREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_PARTITIONSREADER_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
#include <fastrtps/attributes/TopicAttributes.h>
#include <fastrtps/qos/ReaderQos.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/rtps/attributes/ReaderAttributes.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>

#include <ddsrouter_utils/types/Atomicable.hpp>

#include <ddsrouter_core/types/dds/Guid.hpp>
#include <ddsrouter_core/types/dds/SpecificWriterQoS.hpp>

#include <reader/implementations/rtps/CommonReader.hpp>
#include <dynamic/DiscoveryDatabase.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Base PartitionsReader concrete class that implements CommonReader abstract one.
 */
class PartitionsReader : public CommonReader
{
public:

    /**
     * @brief Construct a new PartitionsReader object
     *
     * Get the Attributes and TopicQoS and create the PartitionsReader History and the RTPS PartitionsReader.
     *
     * @param participant_id    Router Id of the Participant that has created this PartitionsReader.
     * @param topic             Topic that this PartitionsReader subscribes to.
     * @param payload_pool      Shared Payload Pool to received data and take it.
     * @param rtps_participant  RTPS Participant pointer (this is not stored).
     *
     * @throw \c InitializationException in case any creation has failed
     */
    PartitionsReader(
            const types::ParticipantId& participant_id,
            const types::DdsTopic& topic,
            std::shared_ptr<PayloadPool> payload_pool,
            fastrtps::rtps::RTPSParticipant* rtps_participant,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

protected:

    types::SpecificWriterQoS specific_qos_of_writer_(const types::Guid& guid) const;
    // void discovered_endpoint_(
    //     const Endpoint& endpoint);

    virtual void fill_received_data_(
        fastrtps::rtps::CacheChange_t* received_change,
        std::unique_ptr<types::DataReceived>& data_to_fill) const noexcept override;

    std::shared_ptr<DiscoveryDatabase> discovery_database_;

};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_RTPS_PARTITIONSREADER_HPP_ */

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
 * @file CommonRTPSRouterParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONRTPSROUTERPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONRTPSROUTERPARTICIPANT_HPP_

#include <fastdds/rtps/participant/ParticipantDiscoveryInfo.h>
#include <fastdds/rtps/reader/ReaderDiscoveryInfo.h>
#include <fastdds/rtps/rtps_fwd.h>
#include <fastdds/rtps/writer/WriterDiscoveryInfo.h>
#include <fastrtps/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipantListener.h>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

#include <participant/IParticipant.hpp>
#include <reader/rtps/Reader.hpp>
#include <writer/rtps/Writer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * TODO
 */
class CommonRTPSRouterParticipant
    : public IParticipant
    , public fastrtps::rtps::RTPSParticipantListener
{
private:

    //! RTPS Participant deleter functor to enable RAII on RTPSParticipant
    struct RTPSParticipantDeleter
    {
        RTPSParticipantDeleter()
        {
        }

        void operator ()(
                fastrtps::rtps::RTPSParticipant* ptr) const;
    };

public:

    /**
     * @brief Illegal constructor that always throws.
     *
     * A RTPS always need a configuration and a discovery_database so this always throws.
     * This constructor exists just to maintain a generic template interface for creating participants.
     *
     * @throw \c InitializationException always.
     */
    CommonRTPSRouterParticipant(
            const types::ParticipantId& id);


    CommonRTPSRouterParticipant(
            const configuration::ParticipantConfiguration& participant_configuration,
            DiscoveryDatabase& discovery_database);

    virtual ~CommonRTPSRouterParticipant();

    virtual void onParticipantDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::ParticipantDiscoveryInfo&& info) override;

    virtual void onReaderDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::ReaderDiscoveryInfo&& info) override;

    virtual void onWriterDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::WriterDiscoveryInfo&& info) override;

protected:

    void create_participant_();

    std::unique_ptr<IWriter> create_writer_(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool) override;

    std::unique_ptr<IReader> create_reader_(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
            DataForwardQueue& data_forward_queue) override;

    template<class DiscoveryInfoKind>
    types::Endpoint create_endpoint_from_info_(
            DiscoveryInfoKind& info);

    /////
    // RTPS specific methods

    virtual fastrtps::rtps::RTPSParticipantAttributes participant_attributes_() const;

    /////
    // VARIABLES
    std::unique_ptr<fastrtps::rtps::RTPSParticipant, RTPSParticipantDeleter> rtps_participant_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONRTPSROUTERPARTICIPANT_HPP_ */

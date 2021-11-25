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

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_COMMONRTPSROUTERPARTICIPANT_IMPL_IPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_COMMONRTPSROUTERPARTICIPANT_IMPL_IPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastrtps/rtps/RTPSDomain.h>

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/SimpleRTPSParticipantConfiguration.hpp>
#include <ddsrouter/participant/implementations/auxiliar/BaseParticipant.hpp>
#include <ddsrouter/reader/implementations/rtps/RTPSRouterReader.hpp>
#include <ddsrouter/writer/implementations/rtps/RTPSRouterWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
template <class ConfigurationType>
class CommonRTPSRouterParticipant : public BaseParticipant<ConfigurationType>
{
public:

    CommonRTPSRouterParticipant(
            const ParticipantConfiguration& participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);
        // : BaseParticipant<ConfigurationType>(participant_configuration, payload_pool, discovery_database);
    // {
    //     DomainId domain = BaseParticipant<ConfigurationType>::configuration_.domain();
    //     fastrtps::rtps::RTPSParticipantAttributes params = participant_attributes();

    //     rtps_participant_ = fastrtps::rtps::RTPSDomain::createParticipant(domain, params);
    //     if (!rtps_participant_)
    //     {
    //         throw InitializationException(utils::Formatter() << "Error creating RTPS Participant " << id());
    //     }

    //     logInfo(DDSROUTER_RTPS_READER,
    //         "New Participant " << configuration_.type() << " created with id " << BaseParticipant<ConfigurationType>::id() <<
    //         " in domain " << domain << " with guid " << rtps_participant_->getGuid());
    // }

    virtual ~CommonRTPSRouterParticipant();

protected:

    std::shared_ptr<IWriter> create_writer_(
            RealTopic topic) override;

    std::shared_ptr<IReader> create_reader_(
            RealTopic topic) override;

    /////
    // RTPS specific methods

    virtual fastrtps::rtps::RTPSParticipantAttributes participant_attributes() const noexcept;

    /////
    // VARIABLES
    fastrtps::rtps::RTPSParticipant* rtps_participant_;

    //! Mutex that guards every access to the RTPS Participant
    mutable std::recursive_mutex rtps_mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/participant/implementations/rtps/impl/CommonRTPSRouterParticipant.ipp>

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_COMMONRTPSROUTERPARTICIPANT_IMPL_IPP_ */

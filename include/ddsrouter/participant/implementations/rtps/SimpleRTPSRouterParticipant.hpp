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
 * @file SimpleRTPSRouterParticipant.hpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_SIMPLERTPSPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_SIMPLERTPSPARTICIPANT_HPP_

#include <fastdds/rtps/rtps_fwd.h>
#include <fastrtps/rtps/attributes/RTPSParticipantAttributes.h>

#include <ddsrouter/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter/configuration/SimpleRTPSParticipantConfiguration.hpp>
#include <ddsrouter/participant/implementations/auxiliar/BaseParticipant.hpp>
#include <ddsrouter/reader/implementations/rtps/RTPSRouterReader.hpp>
#include <ddsrouter/writer/implementations/rtps/RTPSRouterWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Participant with Simple Discovery Protocol.
 *
 * Standard RTPS Participant with Simple Discovery and default attributes.
 */
class SimpleRTPSRouterParticipant : public BaseParticipant<SimpleRTPSParticipantConfiguration>
{
public:

    /**
     * @brief Construct a new Dummy Participant object
     *
     * It uses the \c BaseParticipant constructor.
     * Apart from BaseParticipant, it creates a new RTPSParticipant with default Attributes and domain given
     * by configuration.
     *
     * @throw \c InitializationException in case any internal error has ocurred while creating RTPSParticipant
     * @throw \c IConfigurationException in case configuration was incorrectly set
     */
    SimpleRTPSRouterParticipant(
            const ParticipantConfiguration& participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    /**
     * @brief Destroy Participant and subentities
     *
     * delete RTPSParticipant if set.
     */
    virtual ~SimpleRTPSRouterParticipant();

protected:

    //! Override create_writer_() BaseParticipant method
    std::shared_ptr<IWriter> create_writer_(
            RealTopic topic) override;

    //! Override create_reader_() BaseParticipant method
    std::shared_ptr<IReader> create_reader_(
            RealTopic topic) override;

    /////
    // RTPS specific methods

    /**
     * @brief Default RTPS Router Participant Attributes for this participant
     *
     * @return Participant Attributes
     */
    virtual fastrtps::rtps::RTPSParticipantAttributes participant_attributes() const noexcept;

    /////
    // VARIABLES

    //! RTPS Participant pointer
    fastrtps::rtps::RTPSParticipant* rtps_participant_;

    //! Mutex that guards every access to the RTPS Participant
    mutable std::recursive_mutex rtps_mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_SIMPLERTPSPARTICIPANT_HPP_ */

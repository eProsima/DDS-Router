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
 * @file IParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_IPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_IPARTICIPANT_HPP_

#include <ddsrouter_utils/macros.hpp>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <dynamic/DiscoveryDatabase.hpp>

#include <ddsrouter_core/types/topic/TopicKeyMap.hpp>
#include <participant/IParticipant.hpp>
#include <reader/IReader.hpp>
#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Base Participant that implements common methods for every Participant.
 *
 * In order to inherit from this class, create the protected methods create_writer_ and create_reader_
 *
 * This class stores every Endpoint (Writers and Readers) created by this Participant.
 */
class IParticipant
{

public:

    /**
     * @brief Simplest IParticipant constructor.
     *
     * Data members participant_configuration_ and discovery_database_ are nulled (nullptr).
     *
     * @param Participant ID.
     *
     */
    IParticipant(
            const types::ParticipantId& id);

    /**
     * @brief Generic constructor for a Participant
     *
     * Id and kind are taken from the configuration.
     *
     * @param participant_configuration Configuration for the Participant. Participant Kind is taken from here.
     * @param discovery_database Non-owning reference to Discovery Database
     */
    IParticipant(
            const configuration::ParticipantConfiguration& participant_configuration,
            DiscoveryDatabase& discovery_database);

    /**
     * @brief Destroy the Base Participant object
     *
     * If any writer or reader still exists, removes it and shows a warning
     */
    virtual ~IParticipant();

    /**
     * @brief Return ParticipantId
     */
    const types::ParticipantId& id() const noexcept;

    /**
     * @brief Create Writer and Reader for this topic
     *
     * Create Writer and Reader for this topic which will use a given .
     *
     * @param topic : Input topic
     * @param payload_pool : Associated to the underlyint RTPS Writer and Reader
     * @param  data_forward_queue : On which Reader will deposit DataForward tasks
     *
     * @return A pair of raw pointers representing the creater Writer and Reader.
     */
    std::pair<IWriter*, IReader*> register_topic(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool> payload_pool,
            DataForwardQueue& data_forward_queue);

    /**
     * @brief Enable topic if disabled.
     *
     * @param [in] topic : Topic to be enabled.
     *
     * @return RETCODE_OK if changed from disabled to enabled
     * @return RETCODE_NOT_ENABLED if already enabled
     * @return RETCODE_NOT_ENABLED if reader not associated to topic exist
     */
    utils::ReturnCode enable_topic(
            const types::RealTopic& topic);

    /**
     * @brief Disable topic if enabled.
     *
     * @param [in] topic : Topic to be disabled
     *
     * @return RETCODE_OK if changed from enabled to disabled
     * @return RETCODE_NOT_ENABLED if already disabled
     * @return RETCODE_NOT_ENABLED if reader not associated to topic exist
     */
    utils::ReturnCode disable_topic(
            const types::RealTopic& topic);

protected:

    /**
     * @brief Create a writer object
     *
     * @note Implement this method in every Participant in order to create a class specific Writer
     *
     * @param [in] topic : Topic that this Writer refers to.
     * @return Writer
     */
    virtual std::unique_ptr<IWriter> create_writer_(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool) = 0;

    /**
     * @brief Create a reader object
     *
     * @note Implement this method in every Participant in order to create a class specific Reader
     *
     * @param [in] topic : Topic that this Reader refers to.
     * @return Reader
     */
    virtual std::unique_ptr<IReader> create_reader_(
            const types::RealTopic& topic,
            std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
            DataForwardQueue& data_forward_queue) = 0;

    /////
    // VARIABLES

    //! Owned Participant ID, referenced by Writers and Readers
    const types::ParticipantId id_;

    //! Participant configuration, can be nullptr for some participant types not requiring any configuration
    const configuration::ParticipantConfiguration* configuration_;

    //! DDS Router reference to Discovery Database, can be nullptr for some participant types not requiring any discovery database
    DiscoveryDatabase* discovery_database_;

    //! Writers created by this Participant indexed by topic
    types::TopicKeyMap<std::unique_ptr<IWriter>> writers_;

    //! Readers created by this Participant indexed by topic
    types::TopicKeyMap<std::unique_ptr<IReader>> readers_;
};

/**
 * @brief \c IParticipant to stream serialization
 *
 * This method is merely a to_string of a IParticipant definition.
 * It serialize the Participant ID
 */
std::ostream& operator <<(
        std::ostream& os,
        const IParticipant& participant);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_IPARTICIPANT_HPP_ */

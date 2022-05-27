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
 * @file BaseParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_BASEPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_BASEPARTICIPANT_HPP_

#include <ddsrouter_utils/macros.hpp>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>

#include <participant/IParticipant.hpp>
#include <reader/implementations/auxiliar/BaseReader.hpp>
#include <writer/implementations/auxiliar/BaseWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Abstract Participant that implements generic methods for every Participant.
 *
 * In order to inherit from this class, create the protected methods create_writer_nts_ and create_reader_nts_
 *
 * This class stores every Endpoint created by this Participant.
 */
template <class ConfigurationType>
class BaseParticipant : public IParticipant
{

    // Force ConfigurationType to be subclass of ParticipantConfiguration
    FORCE_TEMPLATE_SUBCLASS(configuration::ParticipantConfiguration, ConfigurationType);

public:

    /**
     * @brief Generic constructor for a Participant
     *
     * Id and kind are taken from the configuration.
     *
     * @param participant_configuration Configuration for the Participant. Participant Kind is taken from here.
     * @param payload_pool DDS Router shared PayloadPool
     * @param discovery_database DDS Router shared Discovery Database
     */
    BaseParticipant(
            const ConfigurationType participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database);

    /**
     * @brief Destroy the Base Participant object
     *
     * If any writer or reader still exists, removes it and shows a warning
     */
    virtual ~BaseParticipant();

    /**
     * @brief Override id() IParticipant method
     *
     * It gets the id from the configuration.
     *
     * Thread safe with mutex \c mutex_ .
     */
    types::ParticipantId id() const noexcept override;

    /**
     * @brief Override kind() IParticipant method
     *
     * It gets the kind from the configuration.
     *
     * Thread safe with mutex \c mutex_ .
     */
    types::ParticipantKind kind() const noexcept override;

    /**
     * @brief Override create_writer() IParticipant method
     *
     * This method calls the protected method \c create_writer_nts_ in order to create the actual Writer.
     * The Writer created is stored in a map.
     *
     * Thread safe with mutex \c mutex_ .
     */
    std::shared_ptr<IWriter> create_writer(
            types::RealTopic topic) override;

    /**
     * @brief Override create_reader() IParticipant method
     *
     * This method calls the protected method \c create_reader_nts_ in order to create the actual Reader.
     * The Reader created is stored in a map.
     *
     * Thread safe with mutex \c mutex_ .
     */
    std::shared_ptr<IReader> create_reader(
            types::RealTopic topic) override;

    /**
     * @brief Override delete_writer() IParticipant method
     *
     * This method calls the protected method \c delete_writer_nts_ in order to delete the Writer.
     *
     * Thread safe with mutex \c mutex_ .
     */
    void delete_writer(
            std::shared_ptr<IWriter> writer) noexcept override;

    /**
     * @brief Override delete_reader() IParticipant method
     *
     * This method calls the protected method \c delete_reader_nts_ in order to delete the Reader.
     *
     * Thread safe with mutex \c mutex_ .
     */
    void delete_reader(
            std::shared_ptr<IReader> reader) noexcept override;

protected:

    /**
     * @brief Create a writer object
     *
     * @note Implement this method in every Participant in order to create a class specific Writer
     * @note This method is called with \c mutex_ locked
     *
     * @param [in] topic : Topic that this Writer refers to.
     * @return Writer
     */
    virtual std::shared_ptr<IWriter> create_writer_nts_(
            types::RealTopic topic) = 0;

    /**
     * @brief Create a reader object
     *
     * @note Implement this method in every Participant in order to create a class specific Reader
     * @note This method is called with \c mutex_ locked
     *
     * @param [in] topic : Topic that this Reader refers to.
     * @return Reader
     */
    virtual std::shared_ptr<IReader> create_reader_nts_(
            types::RealTopic topic) = 0;

    /**
     * @brief Do nothing
     *
     * @note Implement this method in order to delete a class specific Writer
     * @note This method is called with \c mutex_ locked
     *
     * @param [in] writer : Writer to delete
     */
    virtual void delete_writer_nts_(
            std::shared_ptr<IWriter> writer) noexcept;

    /**
     * @brief Do nothing
     *
     * @note Implement this method in order to delete a class specific Reader
     * @note This method is called with \c mutex_ locked
     *
     * @param [in] reader : Reader to delete
     */
    virtual void delete_reader_nts_(
            std::shared_ptr<IReader> reader) noexcept;

    //! Participant configuration
    const ConfigurationType configuration_;

    //! DDS Router shared Payload Pool
    std::shared_ptr<PayloadPool> payload_pool_;

    //! DDS Router shared Discovery Database
    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    //! Writers created by this Participant indexed by topic
    std::map<types::RealTopic, std::shared_ptr<IWriter>> writers_;

    //! Readers created by this Participant indexed by topic
    std::map<types::RealTopic, std::shared_ptr<IReader>> readers_;

    //! Mutex that guards every access to the Participant
    mutable std::mutex mutex_;

    // Allow operator << to use private variables
    template <class C>
    friend std::ostream& operator <<(
            std::ostream&,
            const BaseParticipant<C>&);
};

/**
 * @brief \c BaseParticipant to stream serialization
 *
 * This method is merely a to_string of a BaseParticipant definition.
 * It serialize the Id and kind
 */
template <class ConfigurationType>
std::ostream& operator <<(
        std::ostream& os,
        const BaseParticipant<ConfigurationType>& track);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <participant/implementations/auxiliar/impl/BaseParticipant.ipp>

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_AUXILIAR_BASEPARTICIPANT_HPP_ */

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

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_BASEPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_BASEPARTICIPANT_HPP_

#include <ddsrouter/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter/participant/IParticipant.hpp>
#include <ddsrouter/reader/implementations/auxiliar/BaseReader.hpp>
#include <ddsrouter/types/macros.hpp>
#include <ddsrouter/writer/implementations/auxiliar/BaseWriter.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Abstract Participant that implements generic methods for every Participant.
 *
 * In order to inherit from this class, create the protected methods create_writer_ and create_reader_
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
    ParticipantId id() const noexcept override;

    /**
     * @brief Override kind() IParticipant method
     *
     * It gets the kind from the configuration.
     *
     * Thread safe with mutex \c mutex_ .
     */
    ParticipantKind kind() const noexcept override;

    /**
     * @brief Override create_writer() IParticipant method
     *
     * This method calls the protected method \c create_writer_ in order to create the actual Writer.
     * The Writer created is stored in a map.
     *
     * Thread safe with mutex \c mutex_ .
     */
    std::shared_ptr<IWriter> create_writer(
            RealTopic topic) override;

    /**
     * @brief Override create_reader() IParticipant method
     *
     * This method calls the protected method \c create_reader_ in order to create the actual Reader.
     * The Reader created is stored in a map.
     *
     * Thread safe with mutex \c mutex_ .
     */
    std::shared_ptr<IReader> create_reader(
            RealTopic topic) override;

    /**
     * @brief Override delete_writer() IParticipant method
     *
     * This method calls the protected method \c delete_writer_ in order to delete the Writer.
     *
     * Thread safe with mutex \c mutex_ .
     */
    void delete_writer(
            std::shared_ptr<IWriter> writer) noexcept override;

    /**
     * @brief Override delete_reader() IParticipant method
     *
     * This method calls the protected method \c delete_reader_ in order to delete the Reader.
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
     *
     * @param [in] topic : Topic that this Writer refers to.
     * @return Writer
     */
    virtual std::shared_ptr<IWriter> create_writer_(
            RealTopic topic) = 0;

    /**
     * @brief Create a reader object
     *
     * @note Implement this method in every Participant in order to create a class specific Reader
     *
     * @param [in] topic : Topic that this Reader refers to.
     * @return Reader
     */
    virtual std::shared_ptr<IReader> create_reader_(
            RealTopic topic) = 0;

    /**
     * @brief Do nothing
     *
     * @note Implement this method in order to delete a class specific Writer
     *
     * @param [in] writer : Writer to delete
     */
    virtual void delete_writer_(
            std::shared_ptr<IWriter> writer) noexcept;

    /**
     * @brief Do nothing
     *
     * @note Implement this method in order to delete a class specific Reader
     *
     * @param [in] reader : Reader to delete
     */
    virtual void delete_reader_(
            std::shared_ptr<IReader> reader) noexcept;

    /**
     * @brief Get Id without locking a mutex
     *
     * It gets the id from the configuration.
     *
     * Not Thread safe.
     *
     * @warning this method is called from discovery callbacks, that could be called from Fast DDS while
     * creating an endpoint. So it could lead to a dead lock. That is why it is not thread safe.
     */
    ParticipantId id_nts_() const noexcept;

    //! Participant configuration
    ConfigurationType configuration_;

    //! DDS Router shared Payload Pool
    std::shared_ptr<PayloadPool> payload_pool_;

    //! DDS Router shared Discovery Database
    std::shared_ptr<DiscoveryDatabase> discovery_database_;

    //! Writers created by this Participant indexed by topic
    std::map<RealTopic, std::shared_ptr<IWriter>> writers_;

    //! Readers created by this Participant indexed by topic
    std::map<RealTopic, std::shared_ptr<IReader>> readers_;

    //! Mutex that guards every access to the Participant
    mutable std::recursive_mutex mutex_;

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

} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/participant/implementations/auxiliar/impl/BaseParticipant.ipp>

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_BASEPARTICIPANT_HPP_ */

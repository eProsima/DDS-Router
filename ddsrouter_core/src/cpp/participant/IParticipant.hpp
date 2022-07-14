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

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IDDS_ROUTERPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IDDS_ROUTERPARTICIPANT_HPP_

#include <ddsrouter_core/types/endpoint/Endpoint.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

#include <dynamic/DiscoveryDatabase.hpp>
#include <efficiency/payload/PayloadPool.hpp>
#include <reader/IReader.hpp>
#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Interface that represents a generic Participant as part of a DDSRouter.
 *
 * This class manages the discovery of new remote entities (that do not belong to the router).
 * It also works as a factory for Writers and Readers.
 *
 * Every Participant is associated to a \c ParticipantId that uniquely identifies it.
 * Every Participant is associated with a \c ParticipantKind depending on its implementation.
 *
 * @note In order to implement new Participants, create a subclass of this Interface and implement every method.
 * @note Also it is needed to add the creation of the Participant in the \c ParticipantFactory and a new type of
 * @note \c ParticipantKind .
 */
class IParticipant
{
public:

    /**
     * @brief Return the unique identifier of this Participant.
     *
     * @return This Participant id
     */
    virtual types::ParticipantId id() const noexcept = 0;

    /**
     * @brief Return the Participant kind
     *
     * @return This Participant kind
     */
    virtual types::ParticipantKind kind() const noexcept = 0;

    /**
     * @brief Whether the Participant is repeater or not.
     */
    virtual bool is_repeater() const noexcept = 0;

    /**
     * @brief Return a new Writer
     *
     * Each writer is associated with a \c Bridge with the topic \c topic .
     * This writer will forward messages in this topic.
     *
     * @param [in] topic : Topic that this Writer will work with.
     *
     * @return Writer in this Participant referring this topic
     *
     * @throw \c InitializationException in case the writer creation fails.
     */
    virtual std::shared_ptr<IWriter> create_writer(
            types::RealTopic topic) = 0;

    /**
     * @brief Return a new Reader
     *
     * Each reader is associated with a \c Bridge with the topic \c topic .
     * This reader will receive messages in this topic.
     *
     * @param [in] topic : Topic that this Reader will work with.
     *
     * @return Reader in this Participant referring this topic
     *
     * @throw \c InitializationException in case the reader creation fails.
     */
    virtual std::shared_ptr<IReader> create_reader(
            types::RealTopic topic) = 0;

    /**
     * @brief Delete Writer
     *
     * This method deletes a Writer that has been created by this Participant.
     *
     * @note This method should be able to destroy the Writer as it should not have any other reference.
     *
     * @param [in] writer : Writer to delete
     */
    virtual void delete_writer(
            std::shared_ptr<IWriter> writer) noexcept = 0;

    /**
     * @brief Delete Reader
     *
     * This method deletes a Reader that has been created by this Participant.
     *
     * @note This method should be able to destroy the Reader as it should not have any other reference.
     *
     * @param [in] writer : Reader to delete
     */
    virtual void delete_reader(
            std::shared_ptr<IReader> reader) noexcept = 0;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IDDS_ROUTERPARTICIPANT_HPP_ */

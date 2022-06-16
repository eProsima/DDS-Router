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
 * @file BaseWriterReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_TYPES_ENDPOINT_BASEWRITERREADER_HPP_
#define __SRC_DDSROUTERCORE_TYPES_ENDPOINT_BASEWRITERREADER_HPP_

#include <ddsrouter_core/types/endpoint/Endpoint.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_core/types/topic/Topic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Topmost base class for Writers and Readers (IWriter and IReader) that holds a Participant ID reference and owns topic, with corresponding getters.
 * A reference to an existing ParticipantId is stored instead of an object to have lightweight objects. We can do this because the actual ParticipantId object is stored in the Participant, which will always outlive Writers and Readers.
 *
 */
template <types::EndpointKind EPKind>
class BaseWriterReader
{
public:

    /**
     * @brief Construct a new Writer or Reader object
     *
     * @param id  Participant Id of the Participant that has created this derived Reader or Writer
     * @param id  Topic associated to the derived Reader or Writer
     *
     * @throw \c InitializationException in case any creation has failed
     */
    BaseWriterReader(
            const types::ParticipantId& id,
            const types::RealTopic& topic);

    /**
     * @brief Empty destructor
     */
    virtual ~BaseWriterReader();

    /**
     * @brief Return ParticipantId
     *
     * @return Const reference to ParticipantId
     */
    const types::ParticipantId& id() const noexcept;

    /**
     * @brief Return ParticipantId
     *
     * @return Const reference to topic
     */
    const types::RealTopic& topic() const noexcept;

protected:

    //! Participant ID referencing to ID of its owner IParticipant
    const types::ParticipantId& id_;

    //! Participant topic
    const types::RealTopic topic_;
};

} // namespace types
} // namespace core
} // namespace ddsrouter
} // namespace eprosima

#endif /*__SRC_DDSROUTERCORE_TYPES_ENDPOINT_BASEWRITERREADER_HPP_ */

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
 * @file ParticipantId.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_PARTICIPANTID_HPP_
#define _DDSROUTERCORE_TYPES_PARTICIPANTID_HPP_

#include <string>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * @brief Unique Id for each DDSRouter Participant
 *
 * This class represents the ID that uniquely identifies a Participant inside the DDSRouter.
 * This unique Id is a string that behaves as the key for the unique ID.
 * Two IDs with the same string are the same ID.
 */
class ParticipantId
{
public:

    /**
     * @brief Construct a new Participant invalid Id object
     */
    DDSROUTER_CORE_DllAPI ParticipantId() noexcept;

    /**
     * @brief Construct a new Participant Id object from a string value
     *
     * @param id uniquely identifies the new \c ParticipantId
     */
    DDSROUTER_CORE_DllAPI ParticipantId(
            const std::string& id) noexcept;

    /**
     * @brief Whether this \c ParticipantId is valid
     *
     * For a \c ParticipantId to be invalid, it must contains a string that is not valid as an ID
     *
     * @return true if this Id is valid. False otherwise.
     */
    DDSROUTER_CORE_DllAPI bool is_valid() const noexcept;

    /**
     * @brief Name that uniquely identifies
     *
     * @return key string that represents the ID
     */
    DDSROUTER_CORE_DllAPI std::string id_name() const noexcept;

    /////
    // OPERATOR OVERLOAD

    /**
     * @brief Equality operator
     *
     * Check whether both objects identifies the same participant
     *
     * @param [in] other \c ParticipantId to compare with
     * @return true if this Ids are equal. False otherwise.
     */
    DDSROUTER_CORE_DllAPI bool operator ==(
            const ParticipantId& other) const noexcept;

    DDSROUTER_CORE_DllAPI bool operator !=(
            const ParticipantId& other) const noexcept;

    /**
     * @brief Minor operator
     *
     * Check the internal strings to compare
     *
     * @param [in] other \c ParticipantId to compare with
     * @return true if this Id is lower than \c other . False otherwise.
     */
    DDSROUTER_CORE_DllAPI bool operator <(
            const ParticipantId& other) const noexcept;

    /////
    // STATIC METHODS

    /**
     * @brief Get an invalid \c ParticipantId .
     *
     * @note this method is equal to call default \c ParticipantId constructor.
     *
     * @return ParticipantId invalid
     */
    DDSROUTER_CORE_DllAPI static ParticipantId invalid() noexcept;

    /**
     * @brief Check whether a string is valid to create a \c ParticipantId
     *
     * @param [in] id string as unique key
     * @return true in case the string is valid for an ID. False otherwise
     */
    DDSROUTER_CORE_DllAPI static bool is_valid_id(
            const std::string& id) noexcept;

protected:

    //! Internal string that is the key for the ID
    std::string id_;

    //! Static value that represents an invalid ID string
    static const std::string INVALID_ID; // __invalid_DDSROUTERCORE_participant__
};

//! \c ParticipantId to stream serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const ParticipantId& id);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_PARTICIPANTID_HPP_ */

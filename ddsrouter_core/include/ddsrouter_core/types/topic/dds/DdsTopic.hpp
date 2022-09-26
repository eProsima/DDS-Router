// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file Topic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_DDS_DDSTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_DDS_DDSTOPIC_HPP_

#include <iostream>
#include <string>

#include <ddsrouter_utils/types/Fuzzy.hpp>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/dds/TopicQoS.hpp>
#include <ddsrouter_core/types/topic/Topic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Data struct that represents a DDS Topic of data flow in the Router.
 */
struct DdsTopic : public Topic
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI DdsTopic() = default;

    DDSROUTER_CORE_DllAPI DdsTopic(
            const std::string& topic_name,
            const std::string& type_name) noexcept;

    DDSROUTER_CORE_DllAPI DdsTopic(
            const std::string& topic_name,
            const std::string& type_name,
            const bool keyed,
            const types::TopicQoS& qos) noexcept;

    // DdsTopic& operator=(const DdsTopic& other) noexcept;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    DDSROUTER_CORE_DllAPI bool operator< (const DdsTopic& other) const noexcept;

    DDSROUTER_CORE_DllAPI bool operator== (const DdsTopic& other) const noexcept;

    /////////////////////////
    // METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI virtual bool is_valid(utils::Formatter& error_msg) const noexcept;

    /////////////////////////
    // STATIC METHODS
    /////////////////////////

    DDSROUTER_CORE_DllAPI static bool is_valid_dds_topic(
            const std::string& topic_name,
            const std::string& type_name) noexcept;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Topic Type name
    std::string type_name;

    //! Whether the topic has key or not
    bool keyed = false;

    /**
     * @brief Topic QoS
     *
     * @note This is Fuzzy to solve the case where different QoS for same topic are found
     * (e.g. Writer and Reader default). In these cases the fuzzy level must declare which one should be used.
     * However, as we finally decided that only readers generate bridges, this Fuzzy level is actually not needed (yet).
     */
    utils::Fuzzy<types::TopicQoS> topic_qos{};
};

/**
 * Serialization method for \c DdsTopic object.
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const DdsTopic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_DDS_DDSTOPIC_HPP_ */

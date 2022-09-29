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
 * @file WildcardDdsFilterTopic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_FILTER_WILDCARDDDSFILTERTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_FILTER_WILDCARDDDSFILTERTOPIC_HPP_

#include <iostream>
#include <string>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/filter/DdsFilterTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Data struct that uses wildcards (*, ?) to filter a DDS Router.
 */
struct WildcardDdsFilterTopic : public DdsFilterTopic
{

    /////////////////////////
    // CONSTRUCTORS
    /////////////////////////

    //! Using parent constructos
    using DdsFilterTopic::DdsFilterTopic;

    DDSROUTER_CORE_DllAPI WildcardDdsFilterTopic(
            const std::string topic_name = "*");

    /////////////////////////
    // FILTER METHODS
    /////////////////////////

    //! Implement \c contains parent method.
    DDSROUTER_CORE_DllAPI virtual bool contains(
            const DdsFilterTopic& other) const;

    //! Implement \c contains parent method.
    DDSROUTER_CORE_DllAPI virtual bool matches(
            const DdsTopic& real_topic) const;

    /////////////////////////
    // SERIALIZATION METHODS
    /////////////////////////

    //! Override parent \c serialize method.
    DDSROUTER_CORE_DllAPI virtual std::ostream& serialize(
            std::ostream& os) const override;

    /////////////////////////
    // VARIABLES
    /////////////////////////

    //! Topic name filter
    std::string topic_name;

    //! Type name filter. If not set matches with all.
    utils::Fuzzy<std::string> type_name;

    //! Whether the topic has key or not
    utils::Fuzzy<bool> keyed;
};

/**
 * Serialization method for \c WildcardDdsFilterTopic object.
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const WildcardDdsFilterTopic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_FILTER_WILDCARDDDSFILTERTOPIC_HPP_ */

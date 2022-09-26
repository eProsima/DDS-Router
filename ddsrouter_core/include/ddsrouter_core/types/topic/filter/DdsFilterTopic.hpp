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
 * @file DdsFilterTopic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_FILTER_DDSFILTERTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_FILTER_DDSFILTERTOPIC_HPP_

#include <iostream>
#include <string>

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Generic data struct that represent a filter for a DDS Topic.
 */
struct DDSROUTER_CORE_DllAPI DdsFilterTopic
{

    /////////////////////////
    // OPERATORS
    /////////////////////////

    virtual bool operator< (const DdsFilterTopic& other) const noexcept;

    virtual bool operator== (const DdsFilterTopic& other) const noexcept;

    /////////////////////////
    // FILTER METHODS
    /////////////////////////

    /**
     * Whether this topic filters the same of the topic by argument.
     *
     * This method is used to prevent duplications in filter topic lists.
     * If the topic \c other filters a subset of the topics filtered by \c this, it returns true.
     *
     * Example: {<*>:<*>} contains every DdsFilterTopic.
     * Example: {<>:<>} is contained by every DdsFilterTopic.
     *
     * @param other: Other topic to check if it is contained
     *
     * @return: True if \c other topic filters a subset of \c this
     */
    virtual bool contains(
            const DdsFilterTopic& other) const = 0;

    /**
     * Whether a Real Topic matches the filter of this topic.
     *
     * Virtual method. This method should be implemented in subclasses.
     *
     * @param real_topic: Real topic to check if it is filtered
     *
     * @return: True if \c real_topic matches with \c this filter
     */
    virtual bool matches(
            const DdsTopic& real_topic) const = 0;

    virtual std::ostream& serialize(
        std::ostream& os) const = 0;
};

/**
 * Serialization method for \c DdsFilterTopic object.
 */
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const DdsFilterTopic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_FILTER_DDSFILTERTOPIC_HPP_ */

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
// limitations under the License\.

#pragma once

#include <iostream>
#include <string>

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/interface/ITopic.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

/**
 * Generic data struct that represent a filter for a DDS Topic.
 */
struct DDSPIPE_CORE_DllAPI IFilterTopic
{

    virtual ~IFilterTopic() = default;

    /////////////////////////
    // OPERATORS
    /////////////////////////

    virtual bool operator < (
            const IFilterTopic& other) const noexcept;

    virtual bool operator == (
            const IFilterTopic& other) const noexcept;

    /////////////////////////
    // FILTER METHODS
    /////////////////////////

    /**
     * Whether this topic filters the same of the topic by argument.
     *
     * This method is used to prevent duplications in filter topic lists.
     * If the topic \c other filters a subset of the topics filtered by \c this, it returns true.
     *
     * Example: {<*>:<*>} contains every IFilterTopic.
     * Example: {<>:<>} is contained by every IFilterTopic.
     *
     * @param other: Other topic to check if it is contained
     *
     * @return: True if \c other topic filters a subset of \c this
     */
    virtual bool contains(
            const IFilterTopic& other) const = 0;

    /**
     * Whether a Real Topic matches the filter of this topic.
     *
     * Virtual method. This method should be implemented in subclasses.
     *
     * @param topic: Real topic to check if it is filtered
     *
     * @return: True if \c topic matches with \c this filter
     */
    virtual bool matches(
            const ITopic& topic) const = 0;

    /**
     * @brief Serialize the method in a \c ostream object.
     *
     * This method is the same as \c operator<< .
     * It is implemented to allow virtual \c operator<< .
     *
     * @param [out] os return value of this serialized.
     * @return std::ostream& this serialized inside \c os .
     */
    virtual std::ostream& serialize(
            std::ostream& os) const = 0;
};

/**
 * Serialization method for \c IFilterTopic object.
 */
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const IFilterTopic& t);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

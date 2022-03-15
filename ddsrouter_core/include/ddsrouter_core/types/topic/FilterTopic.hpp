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
 * @file FilterTopic.hpp
 */

#ifndef _DDSROUTERCORE_TYPES_TOPIC_FILTERTOPIC_HPP_
#define _DDSROUTERCORE_TYPES_TOPIC_FILTERTOPIC_HPP_

#include <ddsrouter_core/library/library_dll.h>
#include <ddsrouter_core/types/topic/Topic.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

/**
 * Class that represents a Topic filter
 */
class FilterTopic : public Topic
{
public:

    //! Constructor by topic name, topic type name, and (optionally) has_keyed_set and topic kind
    DDSROUTER_CORE_DllAPI FilterTopic(
            const std::string& topic_name,
            const std::string& topic_type,
            bool has_keyed_set = false,
            bool topic_with_key = false) noexcept;

    // OPERATOR OVERLOAD
    /**
     * Equal operator
     *
     * It compares that topic_name_, topic_type_, topic_with_key_ and has_keyed_set_ are equal
     */
    bool operator ==(
            const FilterTopic& other) const;

    /**
     * Whether this topic filters the same of the topic by argument.
     *
     * This method is used to prevent duplications in filter topic lists.
     * If the topic \c other filters a subset of the topics filtered by \c this, it returns true.
     *
     * Example: {<*>:<*>} contains every FilterTopic.
     * Example: {<>:<>} is contained by every FilterTopic.
     *
     * @param other: Other topic to check if it is contained
     *
     * @return: True if \c other topic filters a subset of \c this
     */
    virtual bool contains(
            const FilterTopic& other) const = 0;

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
            const RealTopic& real_topic) const = 0;

    //! \c has_keyed_set_ getter
    DDSROUTER_CORE_DllAPI bool has_keyed_set() const;

protected:

    //! Whether or not \c topic_with_key_ is explicitly set in yaml configuration
    bool has_keyed_set_;
};

/**
 * TODO
 */
struct RegexTopic : public FilterTopic
{
    using FilterTopic::FilterTopic;
};

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_TOPIC_FILTERTOPIC_HPP_ */

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
 * @file AbstractTopic.hpp
 */

#ifndef _DDS_ROUTER_TYPES_TOPIC_ABSTRACTTOPIC_HPP_
#define _DDS_ROUTER_TYPES_TOPIC_ABSTRACTTOPIC_HPP_

#include <ddsrouter/types/topic/DDSRouterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * Abstract class that represents a Topic filter
 */
class AbstractTopic : public DDSRouterTopic
{
public:

    //! Using parent constructos
    using DDSRouterTopic::DDSRouterTopic;

    //! Destructor
    virtual ~AbstractTopic()
    {
    }

    /**
     * Whether this topic filters the same of the topic by argument.
     *
     * This method is used to prevent duplications in Abstract topic lists.
     * If the topic \c other filters a subset of the topics filteres by \c this, it returns true.
     *
     * Example: {<*>:<*>} contains every AbstractTopic.
     * Example: {<>:<>} is contained by every AbstractTopic.
     *
     * @param other: Other topic to check if it is contained
     *
     * @return: True if \c other topic filters a subset of \c this
     */
    virtual bool contains(
            const AbstractTopic& other) const = 0;

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
};

/**
 * TODO
 */
struct RegexTopic : public AbstractTopic
{
    using AbstractTopic::AbstractTopic;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDS_ROUTER_TYPES_TOPIC_ABSTRACTTOPIC_HPP_ */

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
 * @file AllowedTopicList.hpp
 */

#ifndef _DDSROUTER_TOPIC_ALLOWEDTOPICLIST_HPP_
#define _DDSROUTER_TOPIC_ALLOWEDTOPICLIST_HPP_

#include <list>
#include <map>
#include <mutex>
#include <string>

#include <ddsrouter/types/topic/Topic.hpp>
#include <ddsrouter/types/topic/FilterTopic.hpp>
#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * This object manages the filtering of the topics.
 * It is constituted by a list of allowed topics and a list of blocked topics, and filters each topic
 * by topic name and topic type to see if it is allowed by the lists.
 *
 * In case of an empty allowlist, every topic is allowed except those in blocklist.
 * In case of both lists empty, every topic is allowed.
 */
class AllowedTopicList
{
public:

    //! Default constructor with empty lists
    AllowedTopicList() = default;

    //! Constructor by initialization lists
    AllowedTopicList(
            const std::list<std::shared_ptr<FilterTopic>>& allowlist,
            const std::list<std::shared_ptr<FilterTopic>>& blocklist) noexcept;

    //! Copy constructor. It copies internal lists.
    AllowedTopicList& operator =(
            const AllowedTopicList& other);

    //! Destructor
    virtual ~AllowedTopicList();

    //! Clear all topics in lists
    void clear() noexcept;

    /**
     * Whether topic \c topic is allowed by the lists that constitute this object
     *
     * For a topic to be allowed it must:
     * 1a. Allowlist be empty
     * 1b. Be contained in the allowlist
     * 2. Do not be contained in the blocklist
     *
     * @param topic: topic to check if it is allowed
     *
     * @return True if the topic is allowed, false otherwise
     */
    bool is_topic_allowed(
            const RealTopic& topic) const noexcept;

    /**
     * Equal operator.
     *
     * Two lists are the same if they have the same topics stored.
     *
     * @todo: Two lists are the same when they filter the same topics. Thus, method \c contains in
     * \c FilterTopic must be implemented completely.
     *
     * @param other: other \c AllowedTopicList object to compare with \c this
     *
     * @return True if they are constituted by same topics, false otherwise
     */
    bool operator ==(
            const AllowedTopicList& other) const noexcept;

protected:

    /**
     * @brief Get a list of filtered topics and return a list that filters repeated topics eliminating redundancy
     *
     * @param [in] list: list of topics with redundancy
     * @return Set of topics without redundancy
     */
    static std::set<std::shared_ptr<FilterTopic>> get_topic_list_without_repetition_(
            const std::list<std::shared_ptr<FilterTopic>>& list) noexcept;

    //! List of topics that are not allowed
    std::set<std::shared_ptr<FilterTopic>> blocklist_;

    //! List of topics that are allowed
    std::set<std::shared_ptr<FilterTopic>> allowlist_;

    //! Mutex to restrict access to the class
    mutable std::recursive_mutex mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TOPIC_ALLOWEDTOPICLIST_HPP_ */

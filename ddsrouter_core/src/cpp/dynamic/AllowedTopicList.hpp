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

#ifndef __SRC_DDSROUTERCORE_DYNAMIC_ALLOWEDTOPICLIST_HPP_
#define __SRC_DDSROUTERCORE_DYNAMIC_ALLOWEDTOPICLIST_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <set>

#include <ddsrouter_core/types/topic/Topic.hpp>
#include <ddsrouter_core/types/topic/filter/DdsFilterTopic.hpp>
#include <ddsrouter_core/types/topic/dds/DdsTopic.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

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
            const std::set<std::shared_ptr<types::DdsFilterTopic>>& allowlist,
            const std::set<std::shared_ptr<types::DdsFilterTopic>>& blocklist) noexcept;

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
            const types::DdsTopic& topic) const noexcept;

    /**
     * Equal operator.
     *
     * Two lists are the same if they have the same topics stored.
     *
     * @todo: Two lists are the same when they filter the same topics. Thus, method \c contains in
     * \c DdsFilterTopic must be implemented completely.
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
    static std::set<std::shared_ptr<types::DdsFilterTopic>> get_topic_list_without_repetition_(
            const std::set<std::shared_ptr<types::DdsFilterTopic>>& list) noexcept;

    //! List of topics that are not allowed
    std::set<std::shared_ptr<types::DdsFilterTopic>> blocklist_;

    //! List of topics that are allowed
    std::set<std::shared_ptr<types::DdsFilterTopic>> allowlist_;

    //! Mutex to restrict access to the class
    mutable std::recursive_mutex mutex_;

    // Allow operator << to use private variables
    friend std::ostream& operator <<(
            std::ostream&,
            const AllowedTopicList&);
};

//! \c AllowedTopicList to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const AllowedTopicList& atl);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_TOPIC_ALLOWEDTOPICLIST_HPP_ */

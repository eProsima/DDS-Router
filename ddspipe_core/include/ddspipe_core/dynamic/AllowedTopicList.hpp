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

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <set>

#include <cpp_utils/memory/Heritable.hpp>

#include <ddspipe_core/types/topic/Topic.hpp>
#include <ddspipe_core/types/topic/rpc/RpcTopic.hpp>
#include <ddspipe_core/types/topic/filter/IFilterTopic.hpp>
#include <ddspipe_core/types/topic/dds/DistributedTopic.hpp>

namespace eprosima {
namespace ddspipe {
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
    DDSPIPE_CORE_DllAPI AllowedTopicList();

    //! Constructor by initialization lists
    DDSPIPE_CORE_DllAPI AllowedTopicList(
            const std::set<utils::Heritable<types::IFilterTopic>>& allowlist,
            const std::set<utils::Heritable<types::IFilterTopic>>& blocklist) noexcept;

    //! Copy constructor. It copies internal lists.
    DDSPIPE_CORE_DllAPI AllowedTopicList& operator =(
            const AllowedTopicList& other);

    //! Destructor
    DDSPIPE_CORE_DllAPI virtual ~AllowedTopicList();

    //! Clear all topics in lists
    DDSPIPE_CORE_DllAPI void clear() noexcept;

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
    DDSPIPE_CORE_DllAPI bool is_topic_allowed(
            const ITopic& topic) const noexcept;

    /**
     * Whether RpcTopic \c topic is allowed by the lists that constitute this object
     *
     * For a RpcTopic to be allowed it must:
     * 1. Request topic allowed
     * 2. Reply topic allowed
     *
     * @param topic: topic to check if it is allowed
     *
     * @return True if the topic is allowed, false otherwise
     */
    DDSPIPE_CORE_DllAPI bool is_service_allowed(
            const types::RpcTopic& topic) const noexcept;

    /**
     * Equal operator.
     *
     * Two lists are the same if they have the same topics stored.
     *
     * @todo: Two lists are the same when they filter the same topics. Thus, method \c contains in
     * \c IFilterTopic must be implemented completely.
     *
     * @param other: other \c AllowedTopicList object to compare with \c this
     *
     * @return True if they are constituted by same topics, false otherwise
     */
    DDSPIPE_CORE_DllAPI bool operator ==(
            const AllowedTopicList& other) const noexcept;

protected:

    /**
     * @brief Get a list of filtered topics and return a list that filters repeated topics eliminating redundancy
     *
     * @param [in] list: list of topics with redundancy
     * @return Set of topics without redundancy
     */
    static std::set<utils::Heritable<types::IFilterTopic>> get_topic_list_without_repetition_(
            const std::set<utils::Heritable<types::IFilterTopic>>& list) noexcept;

    //! List of topics that are not allowed
    std::set<utils::Heritable<types::IFilterTopic>> blocklist_;

    //! List of topics that are allowed
    std::set<utils::Heritable<types::IFilterTopic>> allowlist_;

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
} /* namespace ddspipe */
} /* namespace eprosima */

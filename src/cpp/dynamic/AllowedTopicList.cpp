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
 * @file AllowedTopicList.cpp
 *
 */

#include <ddsrouter/dynamic/AllowedTopicList.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: Add logs
AllowedTopicList::AllowedTopicList(
        const std::list<std::shared_ptr<FilterTopic>>& allowlist,
        const std::list<std::shared_ptr<FilterTopic>>& blocklist) noexcept
{
    allowlist_ = AllowedTopicList::get_topic_list_without_repetition_(allowlist);
    blocklist_ = AllowedTopicList::get_topic_list_without_repetition_(blocklist);

    logDebug(DDSROUTER_ALLOWEDTOPICLIST, "New Allowed topic list created:");
    logDebug(DDSROUTER_ALLOWEDTOPICLIST, "New Allowed topic list created: " << *this << ".");
}

AllowedTopicList& AllowedTopicList::operator =(
        const AllowedTopicList& other)
{
    this->allowlist_ = other.allowlist_;
    this->blocklist_ = other.blocklist_;

    return *this;
}

AllowedTopicList::~AllowedTopicList()
{
    // Eliminate all topics
    clear();
}

void AllowedTopicList::clear() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    blocklist_.clear();
    allowlist_.clear();
}

bool AllowedTopicList::is_topic_allowed(
        const RealTopic& topic) const noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // It is accepted by default if allowlist is empty, if not it should pass the allowlist filter
    bool accepted = allowlist_.empty();

    // Check if allowlist filter it (this will do anything if empty and accepted will be true)
    for (std::shared_ptr<FilterTopic> filter : allowlist_)
    {
        if (filter->matches(topic))
        {
            accepted = true;
            break;
        }
    }

    // Check if it has not passed the allowlist so blocklist is skipped
    if (!accepted)
    {
        return false;
    }

    // Allowlist passed, check blocklist
    for (std::shared_ptr<FilterTopic> filter : blocklist_)
    {
        if (filter->matches(topic))
        {
            return false;
        }
    }

    // Blocklist passed, the topic is allowed
    return true;
}

bool AllowedTopicList::operator ==(
        const AllowedTopicList& other) const noexcept
{
    return allowlist_ == other.allowlist_ && blocklist_ == other.blocklist_;
}

std::set<std::shared_ptr<FilterTopic>> AllowedTopicList::get_topic_list_without_repetition_(
        const std::list<std::shared_ptr<FilterTopic>>& list) noexcept
{
    std::set<std::shared_ptr<FilterTopic>> non_repeated_list;

    // Store each topic without repetition
    for (std::shared_ptr<FilterTopic> new_topic : list)
    {
        bool add_it = true;
        std::set<std::shared_ptr<FilterTopic>> repeated;

        // Check if it is contained or contains any topic already in the list
        for (std::shared_ptr<FilterTopic> topic_stored : non_repeated_list)
        {
            if (topic_stored->contains(*new_topic))
            {
                // It is repeated, so it must not be added
                add_it = false;
                break;
            }
            else if (new_topic->contains(*topic_stored))
            {
                // There is a repeated topic stored, pop it and add this one instead
                repeated.insert(topic_stored);
            }
        }

        // Remove topics repeated
        for (std::shared_ptr<FilterTopic> topic_repeated : repeated)
        {
            non_repeated_list.erase(topic_repeated);
        }

        // Add new topic if it is not repeated
        if (add_it)
        {
            non_repeated_list.insert(new_topic);
        }
    }

    return non_repeated_list;
}

std::ostream& operator <<(
        std::ostream& os,
        const AllowedTopicList& atl)
{
    os << "AllowedTopicList{";

    // Allowed topics
    os << "allowed";
    utils::container_to_stream<std::shared_ptr<FilterTopic>, true>(os, atl.allowlist_);

    // Blocked topics
    os << "blocked";
    utils::container_to_stream<std::shared_ptr<FilterTopic>, true>(os, atl.blocklist_);

    os << "}";

    return os;
}

} /* namespace ddsrouter */
} /* namespace eprosima */

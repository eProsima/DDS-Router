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

#include <databroker/dynamic/AllowedTopicList.hpp>
#include <databroker/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace databroker {

// TODO: Add logs
AllowedTopicList::AllowedTopicList(
        const std::list<std::shared_ptr<AbstractTopic>>& whitelist,
        const std::list<std::shared_ptr<AbstractTopic>>& blacklist)
{
    whitelist_ = AllowedTopicList::get_topic_list_without_repetition_(whitelist);
    blacklist_ = AllowedTopicList::get_topic_list_without_repetition_(blacklist);
}

AllowedTopicList::~AllowedTopicList()
{
}

void AllowedTopicList::clear()
{
    blacklist_.clear();
    whitelist_.clear();
}

bool AllowedTopicList::is_topic_allowed(
        const RealTopic& topic) const
{
    // It is accepted by default if whitelist is empty, if no is should pass the whitelist filter
    bool accepted = whitelist_.empty();

    // Check if whitelist filter it (this will do anything if empty and accepted will be true)
    for (std::shared_ptr<AbstractTopic> filter : whitelist_)
    {
        if (filter->matches(topic))
        {
            accepted = true;
            break;
        }
    }

    // Check if it has not passed the whitelist so blacklist is skipped
    if (!accepted)
    {
        return false;
    }

    // Whitelist passed, check blacklist
    for (std::shared_ptr<AbstractTopic> filter : blacklist_)
    {
        if (filter->matches(topic))
        {
            return false;
        }
    }

    // Blacklist passed, the topic is allowed
    return true;
}

bool AllowedTopicList::operator ==(
        const AllowedTopicList& other) const
{
    return whitelist_ == other.whitelist_ && blacklist_ == other.blacklist_;
}

std::set<std::shared_ptr<AbstractTopic>> AllowedTopicList::get_topic_list_without_repetition_(
        const std::list<std::shared_ptr<AbstractTopic>>& list)
{
    std::set<std::shared_ptr<AbstractTopic>> non_repeated_list;

    // Store each topic without repetition
    for (std::shared_ptr<AbstractTopic> new_topic : list)
    {
        bool add_it = true;
        std::set<std::shared_ptr<AbstractTopic>> repeated;

        // Check if it is contained or contains any topic already in the list
        for (std::shared_ptr<AbstractTopic> topic_stored : non_repeated_list)
        {
            if (topic_stored->contains(*new_topic))
            {
                // It is repeted, so it must not be added
                add_it = true;
                break;
            }
            else if (new_topic->contains(*topic_stored))
            {
                // There is a repeated topic stored, pop it and add this instead
                repeated.insert(topic_stored);
            }
        }

        // Remove topics repeated
        for (std::shared_ptr<AbstractTopic> topic_repeated : repeated)
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

} /* namespace databroker */
} /* namespace eprosima */

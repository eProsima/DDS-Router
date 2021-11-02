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
    : whitelist_(whitelist)
    , blacklist_(blacklist)
{
}

AllowedTopicList::~AllowedTopicList()
{
}

void AllowedTopicList::clear()
{
    // TODO
    throw UnsupportedException("AllowedTopicList::clear not supported yet");
}

bool AllowedTopicList::is_topic_allowed(
        const RealTopic& topic) const
{
    // TODO
    throw UnsupportedException("AllowedTopicList::is_topic_allowed not supported yet");
}

bool AllowedTopicList::operator ==(
        const AllowedTopicList& other) const
{
    // TODO
    throw UnsupportedException("AllowedTopicList::operator== not supported yet");
}

} /* namespace databroker */
} /* namespace eprosima */

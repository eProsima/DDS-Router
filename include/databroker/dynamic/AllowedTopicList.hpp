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

#ifndef _DATABROKER_TOPIC_ALLOWEDTOPICLIST_HPP_
#define _DATABROKER_TOPIC_ALLOWEDTOPICLIST_HPP_

#include <list>
#include <map>
#include <string>

#include <databroker/types/topic/DatabrokerTopic.hpp>
#include <databroker/types/topic/AbstractTopic.hpp>
#include <databroker/types/topic/RealTopic.hpp>
#include <databroker/types/ReturnCode.hpp>
#include <databroker/types/RawConfiguration.hpp>

namespace eprosima {
namespace databroker {

/**
 * TODO
 */
class AllowedTopicList
{
public:
    // Allow all topics by default
    AllowedTopicList();

    AllowedTopicList(
            bool allow_topics_by_default,
            const std::vector<AbstractTopic>& whitelist,
            const std::vector<AbstractTopic>& blacklist); // blocked_topics or allowed_topics depending on use_whitelist

    ReturnCode clear();

    ReturnCode block_topic(
            const AbstractTopic& new_topic);

    ReturnCode allow_topic(
            const AbstractTopic& topic);

    bool is_topic_allowed(
            const RealTopic& topic) const;

    bool are_topics_allowed_by_default() const;

    ReturnCode allow_topics_by_default(
            bool status);

    ReturnCode reload(
            const std::list<AbstractTopic*>& whitelist,
            const std::list<AbstractTopic*>& blacklist);

protected:

    std::list<AbstractTopic*> blacklist_;

    std::list<AbstractTopic*> whitelist_;

    bool allow_topics_by_default_,
};

} /* namespace databroker */
} /* namespace eprosima */

#endif /* _DATABROKER_TOPIC_ALLOWEDTOPICLIST_HPP_ */

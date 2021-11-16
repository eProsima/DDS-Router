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
 * @file DiscoveryDatabase.hpp
 */

#ifndef _DDSROUTER_DYNAMIC_DISCOVERYDATABASE_HPP_
#define _DDSROUTER_DYNAMIC_DISCOVERYDATABASE_HPP_

#include <string>
#include <map>

#include <ddsrouter/types/topic/RealTopic.hpp>
#include <ddsrouter/types/endpoint/Guid.hpp>
#include <ddsrouter/types/endpoint/Endpoint.hpp>
#include <ddsrouter/types/ReturnCode.hpp>

namespace eprosima {
namespace ddsrouter {

/**
 * TODO
 */
class DiscoveryDatabase
{
public:

    bool topic_exists(
            const RealTopic& topic);

    bool endpoint_exists(
            const Guid& guid);

    ReturnCode add_or_modify_endpoint(
            const Endpoint& new_endpoint);

    ReturnCode erase_endpoint(
            const Guid& guid_of_endpoint_to_erase);

    ReturnCode erase_endpoint(
            const Endpoint& endpoint_to_erase);

    // TODO
    // some way of allowing participants to subscribe to a callback when new information arrives

protected:

    std::map<Guid, Endpoint> entities_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_DYNAMIC_DISCOVERYDATABASE_HPP_ */

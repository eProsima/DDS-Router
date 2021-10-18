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

#ifndef _DATABROKER_DYNAMIC_DISCOVERYDATABASE_HPP_
#define _DATABROKER_DYNAMIC_DISCOVERYDATABASE_HPP_

#include <string>
#include <map>

#include <databroker/dynamic/Entities.hpp>
#include <databroker/dynamic/QoS.hpp>
#include <databroker/dynamic/Topic.hpp>
#include <databroker/types/Guid.hpp>

namespace eprosima {
namespace databroker {

class DiscoveryDatabase
{
public:

    bool topic_exists(const Topic& topic);

    bool entity_exists(const Guid& guid);

    ReturnCode add_entity(
        const Entity& new_entity);

    ReturnCode modify_entity(
        const Entity& new_entity);

    ReturnCode erase_entity(
        const Guid& guid_of_entity_to_erase);

    ReturnCode erase_entity(
        const Entity& entity_to_erase);

protected:

    std::map<Guid, Entity> entities_;

    std::vector<Topic> topics_;
};

} /* namespace rtps */
} /* namespace databroker */

#endif /* _DATABROKER_DYNAMIC_DISCOVERYDATABASE_HPP_ */

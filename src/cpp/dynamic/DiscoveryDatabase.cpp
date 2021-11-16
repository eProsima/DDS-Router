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
 * @file DiscoveryDatabase.cpp
 *
 */

#include <ddsrouter/dynamic/DiscoveryDatabase.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>

namespace eprosima {
namespace ddsrouter {

bool DiscoveryDatabase::topic_exists(
        const RealTopic& topic) const
{
    // TODO
    throw UnsupportedException("DiscoveryDatabase::topic_exists is not implemented yet");
}

bool DiscoveryDatabase::endpoint_exists(
        const Guid& guid) const
{
    // TODO
    throw UnsupportedException("DiscoveryDatabase::endpoint_exists is not implemented yet");
}

ReturnCode DiscoveryDatabase::add_or_modify_endpoint(
        const Endpoint& new_endpoint)
{
    // TODO
    throw UnsupportedException("DiscoveryDatabase::add_or_modify_endpoint is not implemented yet");
}

ReturnCode DiscoveryDatabase::erase_endpoint(
        const Guid& guid_of_endpoint_to_erase)
{
    // TODO
    throw UnsupportedException("DiscoveryDatabase::erase_endpoint is not implemented yet");
}

ReturnCode DiscoveryDatabase::erase_endpoint(
        const Endpoint& endpoint_to_erase)
{
    // TODO
    throw UnsupportedException("DiscoveryDatabase::erase_endpoint is not implemented yet");
}

Endpoint DiscoveryDatabase::get_endpoint(
    const Guid& guid_of_endpoint_to_erase) const
{
    // TODO
    throw UnsupportedException("DiscoveryDatabase::get_endpoint is not implemented yet");
}

} /* namespace ddsrouter */
} /* namespace eprosima */

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

#include <ddspipe_core/testing/random_values.hpp>

#include <ddspipe_participants/testing/random_values.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace testing {

using namespace eprosima::ddspipe::participants::types;

Address random_address(
        unsigned int seed /* = 0 */)
{
    return Address("127.0.0.1", seed, seed, TransportProtocol::udp);
}

DiscoveryServerConnectionAddress random_connection_address(
        unsigned int seed /* = 0 */,
        unsigned int size /* = 1 */,
        bool ros /* = false */)
{
    std::set<Address> addresses;
    for (unsigned int i = 0; i < size; ++i)
    {
        addresses.insert(
            random_address((seed + i)));
    }

    return DiscoveryServerConnectionAddress(
        core::testing::random_guid_prefix(seed, ros),
        addresses
    );
}

} /* namespace testing */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */

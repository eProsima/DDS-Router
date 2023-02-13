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

#include <ddspipe_core/testing/test_utils.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace test {

using namespace eprosima::ddspipe::core::types;

Guid random_guid(
        uint16_t seed /* = 1 */)
{
    Guid guid;
    guid.entityId.value[3] = static_cast<eprosima::fastrtps::rtps::octet>(seed);
    guid.guidPrefix.value[0] = 0x01;
    guid.guidPrefix.value[1] = 0x0f;
    return guid;
}

DomainId random_domain(
        uint16_t seed /* = 0 */)
{
    return DomainId(static_cast<DomainIdType>(seed));
}

GuidPrefix random_guid_prefix(
        uint16_t seed /* = 0 */,
        bool ros /* = false */)
{
    if (ros)
    {
        return GuidPrefix(true, seed);
    }
    else
    {
        return GuidPrefix(static_cast<uint32_t>(seed));
    }
}

ParticipantId random_participant_id(
        uint16_t seed /* = 0 */)
{
    std::vector<std::string> names = {
        "participant",
        "PART_1",
        "echo",
        "Barro_p",
    };

    return ParticipantId(names[seed % names.size()] + std::to_string(seed));
}

} /* namespace test */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

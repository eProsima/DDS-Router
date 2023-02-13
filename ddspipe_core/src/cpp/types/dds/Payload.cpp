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

#include <sstream>

#include <ddspipe_core/types/dds/Payload.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

std::ostream& operator <<(
        std::ostream& os,
        const eprosima::fastrtps::rtps::octet& octet)
{
    os << std::hex << std::setfill('0') << std::setw(2) << static_cast<uint16_t>(octet) << std::dec;
    return os;
}

std::ostream& operator <<(
        std::ostream& os,
        const Payload& payload)
{
    os << "Payload{";

    for (uint32_t i = 0; (payload.length != 0) && (i < (payload.length - 1)); ++i)
    {
        os << payload.data[i] << " ";
    }

    // Avoid printing extra space after last byte
    if (payload.length > 0)
    {
        os << payload.data[payload.length - 1];
    }

    os << "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

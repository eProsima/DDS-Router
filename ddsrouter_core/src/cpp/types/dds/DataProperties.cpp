// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DataProperties.cpp
 *
 */

#include <ddsrouter_core/types/dds/DataProperties.hpp>
#include <ddsrouter_utils/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

bool DataProperties::operator < (
        const DataProperties& other) const noexcept
{
    if (this->writer_qos < other.writer_qos)
    {
        return true;
    }
    else if (this->writer_qos == other.writer_qos)
    {
        return this->instanceHandle < other.instanceHandle;
    }

    // NOTE: operator> does not exist, so use == instead
    // else => this->writer_qos > other.writer_qos
    return false;

}

bool DataProperties::operator == (
        const DataProperties& other) const noexcept
{
    // NOTE: Ownership not supported
    return this->writer_qos == other.writer_qos && this->instanceHandle == other.instanceHandle;
}

std::ostream& operator <<(
        std::ostream& os,
        const DataProperties& qos)
{
    os <<
        "DataProperties{" << qos.writer_qos <<
        ";" << qos.instanceHandle <<
        "}";

    return os;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

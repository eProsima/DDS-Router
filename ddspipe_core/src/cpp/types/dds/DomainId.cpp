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

#include <ddspipe_core/types/dds/DomainId.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

DomainId::DomainId (
        bool discovery_server) noexcept
    : domain_id(DEFAULT_DOMAIN_ID)
{
    // Discovery Server case has a different default value
    if (discovery_server)
    {
        domain_id = DEFAULT_DISCOVERY_SERVER_DOMAIN_ID;
    }
}

DomainId::operator DomainIdType() const noexcept
{
    return this->domain_id;
}

bool DomainId::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    if (domain_id > MAX_DOMAIN_ID)
    {
        error_msg << "Domain could not be higher than " << MAX_DOMAIN_ID << ". ";
        return false;
    }
    return true;
}

std::ostream& operator <<(
        std::ostream& output,
        const DomainId& domain)
{
    output << "DomainId{" << domain.domain_id << "}";
    return output;
}

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

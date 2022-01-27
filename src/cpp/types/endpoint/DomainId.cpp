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
 * @file DomainId.cpp
 *
 */

#include <ddsrouter/types/endpoint/DomainId.hpp>

namespace eprosima {
namespace ddsrouter {

const DomainIdType DomainId::DEFAULT_DOMAIN_ID_ = 0;
const DomainIdType DomainId::DEFAULT_DISCOVERY_SERVER_DOMAIN_ID_ = 66;
const DomainIdType DomainId::MAX_DOMAIN_ID_ = 250;

DomainId::DomainId (
        bool discovery_server /*= false*/) noexcept
    : domain_id_(DEFAULT_DOMAIN_ID_)
{
    // Discovery Server case has a different default value
    if (discovery_server)
    {
        domain_id_ = DEFAULT_DISCOVERY_SERVER_DOMAIN_ID_;
    }
}

DomainId::DomainId (
        const DomainIdType& domain) noexcept
    : domain_id_(domain)
{
}

DomainIdType DomainId::operator ()() const noexcept
{
    return domain_id_;
}

DomainIdType DomainId::domain_id() const noexcept
{
    return domain_id_;
}

bool DomainId::is_valid() const noexcept
{
    return domain_id_ <= MAX_DOMAIN_ID_;
}

bool DomainId::operator ==(
        const DomainId& other) const noexcept
{
    return this->domain_id() == other.domain_id();
}

std::ostream& operator <<(
        std::ostream& output,
        const DomainId& domain)
{
    output << "DomainId{" << domain.domain_id() << "}";
    return output;
}

} /* namespace ddsrouter */
} /* namespace eprosima */

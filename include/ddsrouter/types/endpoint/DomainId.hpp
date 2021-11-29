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
 * @file DomainId.hpp
 */

#ifndef _DDSROUTER_TYPES_DDSTYPES_HPP_
#define _DDSROUTER_TYPES_DDSTYPES_HPP_

#include <fastrtps/types/TypesBase.h>

#include <ddsrouter/types/RawConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {

// Use FastDDS Domain Id type
using DomainIdType = eprosima::fastdds::dds::DomainId_t;

class DomainId
{
public:

    DomainId (bool discovery_server = false) noexcept;

    DomainId (const DomainIdType& domain) noexcept;

    DomainIdType operator ()() const noexcept;

    DomainIdType domain_id() const noexcept;

    /////
    // YAML methods
    DomainId (const RawConfiguration& configuration);

    RawConfiguration dump() const; // TODO: Once implemented add noexcept

protected:

    DomainIdType domain_id_;

    static const DomainIdType DEFAULT_DOMAIN_ID_;
    static const DomainIdType DEFAULT_DISCOVERY_SERVER_DOMAIN_ID_;
};

std::ostream& operator <<(
        std::ostream& output,
        const DomainId& domain);

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TYPES_DDSTYPES_HPP_ */

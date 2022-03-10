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

#ifndef _DDSROUTERCORE_TYPES_DDS_DOMAINID_HPP_
#define _DDSROUTERCORE_TYPES_DDS_DOMAINID_HPP_

#include <fastrtps/types/TypesBase.h>

#include <ddsrouter_core/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {

// Use FastDDS Domain Id type
using DomainIdType = eprosima::fastdds::dds::DomainId_t;

/**
 * @brief RTPS Domain ID.
 */
class DomainId
{
public:

    /**
     * @brief Domain ID by default for DDS Routers
     *
     * In case of Discovery Server, default domain is 66
     * In case of Simple Discovery, default domain is 0
     *
     * @param discovery_server
     */
    DDSROUTER_CORE_DllAPI DomainId (
            bool discovery_server = false) noexcept;

    //! Standar constructor by number
    DDSROUTER_CORE_DllAPI DomainId (
            const DomainIdType& domain) noexcept;

    //! Return Fast DDS value for Domain ID
    DDSROUTER_CORE_DllAPI DomainIdType operator ()() const noexcept;

    //! Return Fast DDS value for Domain ID
    DDSROUTER_CORE_DllAPI DomainIdType domain_id() const noexcept;

    DDSROUTER_CORE_DllAPI bool is_valid() const noexcept;

    DDSROUTER_CORE_DllAPI bool operator ==(
            const DomainId& other) const noexcept;

protected:

    //! Value of Fast DDS Domain ID
    DomainIdType domain_id_;

    //! Default value for Simple Discovery
    static const DomainIdType DEFAULT_DOMAIN_ID_;                   // 0

    //! Default value for Discovery Server
    static const DomainIdType DEFAULT_DISCOVERY_SERVER_DOMAIN_ID_;  // 66

    //! Maximum Domain Id valid
    static const DomainIdType MAX_DOMAIN_ID_;  // 250
};

//! \c DomainId serializator
DDSROUTER_CORE_DllAPI std::ostream& operator <<(
        std::ostream& output,
        const DomainId& domain);

} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_TYPES_DDS_DOMAINID_HPP_ */

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
// limitations under the License\.

#pragma once

#include <fastrtps/types/TypesBase.h>

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/configuration/IConfiguration.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

// Use FastDDS Domain Id type
using DomainIdType = eprosima::fastdds::dds::DomainId_t;

/**
 * @brief RTPS Domain ID.
 */
struct DomainId : public IConfiguration
{

    DDSPIPE_CORE_DllAPI DomainId () = default;

    DDSPIPE_CORE_DllAPI DomainId (bool discovery_server) noexcept;

    DDSPIPE_CORE_DllAPI operator DomainIdType() const noexcept;

    DDSPIPE_CORE_DllAPI virtual bool is_valid(
            utils::Formatter& error_msg) const noexcept override;

    //! Value of Fast DDS Domain ID
    DomainIdType domain_id = DEFAULT_DOMAIN_ID;

    /////////////////////////
    // STATIC CONST VALUES
    /////////////////////////

    //! Default value for Simple Discovery
    static constexpr const DomainIdType DEFAULT_DOMAIN_ID = 0;

    //! Default value for Discovery Server
    static constexpr const DomainIdType DEFAULT_DISCOVERY_SERVER_DOMAIN_ID = 66;

    //! Maximum Domain Id valid
    static constexpr const DomainIdType MAX_DOMAIN_ID = 232;
};

//! \c DomainId serializator
DDSPIPE_CORE_DllAPI std::ostream& operator <<(
        std::ostream& output,
        const DomainId& domain);

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

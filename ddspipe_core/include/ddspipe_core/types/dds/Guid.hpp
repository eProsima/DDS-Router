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

#include <fastrtps/rtps/common/Guid.h>

#include <ddspipe_core/library/library_dll.h>
#include <ddspipe_core/types/dds/GuidPrefix.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace types {

//! Unique Id of every Endpoint
class DDSPIPE_CORE_DllAPI Guid : public fastrtps::rtps::GUID_t
{
public:

    //! Using parent constructors
    using fastrtps::rtps::GUID_t::GUID_t;

    //! Default constructor
    Guid() = default;

    //! Copy constructor
    Guid(
            const fastrtps::rtps::GUID_t& x);

    //! Move constructor
    Guid(
            fastrtps::rtps::GUID_t&& x);

    //! Equal operator (inherited from GUID_t)
    Guid& operator = (
            const fastrtps::rtps::GUID_t& other) noexcept;

    /**
     * Whether the guid is a valid one
     *
     * To be valid, the GuidPrefix and the EntityId must not be invalid / unknown
     */
    bool is_valid() const noexcept;

    //! Return GuidPrefix from this Guid
    GuidPrefix guid_prefix() const noexcept;
};

} /* namespace types */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

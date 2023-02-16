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

#pragma once

#include <ddspipe_core/interface/IWriter.hpp>

#include <ddspipe_participants/library/library_dll.h>

namespace eprosima {
namespace ddspipe {
namespace participants {

/**
 * Writer that has an empty implementation.
 * It does not send anything.
 */
DDSPIPE_PARTICIPANTS_DllAPI class BlankWriter : public core::IWriter
{
public:

    //! Override enable() IWriter method
    void enable() noexcept override;

    //! Override disable() IWriter method
    void disable() noexcept override;

    //! Override write() IWriter method
    utils::ReturnCode write(
            core::IRoutingData& data) noexcept override;
};

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */

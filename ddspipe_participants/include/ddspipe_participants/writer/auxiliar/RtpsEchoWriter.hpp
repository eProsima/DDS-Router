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

#include <atomic>

#include <ddspipe_core/types/participant/ParticipantId.hpp>
#include <ddspipe_core/types/topic/dds/DdsTopic.hpp>

#include <ddspipe_participants/library/library_dll.h>
#include <ddspipe_participants/writer/auxiliar/BlankWriter.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {

/**
 * Writer Implementation that prints in stdout every message that is required to write.
 */
class RtpsEchoWriter : public BlankWriter
{
public:

    //! Using parent class constructors
    DDSPIPE_PARTICIPANTS_DllAPI RtpsEchoWriter(
            const core::types::DdsTopic& topic,
            bool verbose);

protected:

    /**
     * @brief Print data in a human friendly way.
     *
     * @param data : data to print
     * @return RETCODE_OK always
     */
    virtual utils::ReturnCode write(
            core::IRoutingData& data) noexcept override;

    //! Topic that this Writer refers to
    core::types::DdsTopic topic_;

    // Specific enable/disable do not need to be implemented
    bool verbose_;
};

} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */

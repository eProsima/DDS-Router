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
 * @file EchoWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_ECHOWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_ECHOWRITER_HPP_

#include <atomic>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

#include <writer/implementations/auxiliar/BlankWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Writer Implementation that prints in stdout every message that is required to write.
 */
class EchoWriter : public BlankWriter
{
public:

    //! Using parent class constructors
    EchoWriter(
            const types::RealTopic& topic,
            bool verbose);

protected:

    /**
     * @brief Print data in a human friendly way.
     *
     * @param data : data to print
     * @return RETCODE_OK always
     */
    virtual utils::ReturnCode write(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

    // Specific enable/disable do not need to be implemented

    bool verbose_;

    //! Topic that this Writer refers to
    types::RealTopic topic_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_ECHOWRITER_HPP_ */

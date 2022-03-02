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

#include <writer/implementations/auxiliar/BaseWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Writer Implementation that prints in stdout every message that is required to write.
 */
class EchoWriter : public BaseWriter
{
public:

    //! Using parent class constructors
    using BaseWriter::BaseWriter;

protected:

    /**
     * @brief Print data in a human friendly way.
     *
     * @param data : data to print
     * @return RETCODE_OK always
     */
    virtual ReturnCode write_(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

    // Specific enable/disable do not need to be implemented
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_ECHOWRITER_HPP_ */

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

#include <cpp_utils/ReturnCode.hpp>

#include <ddspipe_core/interface/IRoutingData.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {

/**
 * Interface that represents a generic Writer as part of a DDSRouter.
 *
 * This class manages the sending of data to remote endpoints in a specific topic.
 *
 * @note In order to implement new Writers, create a subclass of this Interface and implement every method.
 * @note Also it is needed to add the creation of the Writer in the Participant required.
 *
 * Writers will start being disabled.
 */
DDSPIPE_CORE_DllAPI class IWriter
{
public:

    /**
     * @brief Virtual dtor to allow inheritance.
     */
    virtual ~IWriter() = default;

    /**
     * @brief Enable Writer
     *
     * A Writer enabled can send messages.
     *
     * By default the Writer is disabled. Call this method to activate it.
     */
    virtual void enable() noexcept = 0;

    /**
     * @brief Disable Writer
     *
     * A Writer disabled does not send data.
     * @note Method \c write should never be called from a disabled writer
     */
    virtual void disable() noexcept = 0;

    /**
     * @brief Asynchronously write a message for remote endpoints
     *
     * This method will take the data received by another Participant's Reader and should send it forward.
     * In the variable \c data there is the \c Guid of the Endpoint that originally sent the data.
     * In \c data there is also the payload of the data. This payload should be copied/moved from the DDSRouter's
     * PayloadPool to the Writer's PayloadPool (in case it is the same Pool, the data will not be copied).
     * The writer should write asynchronously. It should take the data and exit the function,
     * and internally send the data.
     *
     * Once the data has been sent, the Writer should release the payload.
     *
     * @param [in] data : object containing the payload to be sent
     *
     * @return \c RETCODE_OK if the data has been written correctly
     * @return \c RETCODE_ERROR if there has been any error while writing the sample
     * @return \c RETCODE_NOT_ENABLED if the writer is not enabled (this should not happen)
     */
    virtual utils::ReturnCode write(
            IRoutingData& data) noexcept = 0;
};

} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

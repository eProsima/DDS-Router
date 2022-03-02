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
 * @file IReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IDDS_ROUTERREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IDDS_ROUTERREADER_HPP_

#include <functional>

#include <ddsrouter_utils/ReturnCode.hpp>

#include <ddsrouter_core/types/dds/Data.hpp>
#include <ddsrouter_core/types/topic/RealTopic.hpp>

#include <efficiency/PayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Interface that represents a generic Reader as part of a DDSRouter.
 *
 * This class manages the reception of new remote data in a specific topic.
 * It also calls the \c Track on_data_available callback whenever new data is received.
 *
 * @note In order to implement new Readers, create a subclass of this Interface and implement every method.
 * @note Also it is needed to add the creation of the Reader in the Participant required.
 *
 * Readers will start being disabled.
 */
class IReader
{
public:

    /**
     * @brief Enable Reader
     *
     * A Reader enabled will call on_data_available callback whenever new data is received.
     *
     * By default the Reader is disabled. Call this method to activate it.
     */
    virtual void enable() noexcept = 0;

    /**
     * @brief Disable Reader
     *
     * A disabled Reader does not call on_data_available callback whenever new data is received.
     * @note Disabling a Reader does not mean that messages should not arrive, that depends on the implementation.
     *
     * @warning: This method should stop calling the callback \c on_data_available_lambda if more data arrives while
     * disabled.
     */
    virtual void disable() noexcept = 0;

    /**
     * @brief Set the callback that should be called whenever a new message arrives
     *
     * Each Reader is associated with one \c Track . This Track has a callback that should be called whenever the
     * Reader receives new data. This function sets this callback for the Reader.
     *
     * @param [in] on_data_available_lambda : \c Track callback
     */
    virtual void set_on_data_available_callback(
            std::function<void()> on_data_available_lambda) noexcept = 0;

    /**
     * @brief Unset the callback that should be called whenever a new message arrives.
     *
     * After this method, the Reader should not notify any message that arrives.
     */
    virtual void unset_on_data_available_callback() noexcept = 0;

    /**
     * @brief Take oldest received message from the Reader
     *
     * This method will take the oldest sample received by the Reader and will set it to the argument \c data
     * in a way that the payload in \c data must be inside the DDS Router PayloadPool.
     * In \c data,  the Guid of the Endpoint that originally sent this data must be specified.
     *
     * @param [out] data : object where the payload received should be copied (referenced)
     *
     * @return \c RETCODE_OK if the data has been taken correctly
     * @return \c RETCODE_NO_DATA if there is no more data to take
     * @return \c RETCODE_ERROR if there has been any error while taking a sample
     * @return \c RETCODE_NOT_ENABLED if the reader is not enabled (this should not happen)
     */
    virtual utils::ReturnCode take(
            std::unique_ptr<types::DataReceived>& data) noexcept = 0;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IDDS_ROUTERREADER_HPP_ */

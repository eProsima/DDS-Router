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
 * @file MockWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_MOCKWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_MOCKWRITER_HPP_

#include <condition_variable>
#include <mutex>

#include <ddsrouter_event/wait/CounterWaitHandler.hpp>

#include <writer/implementations/auxiliar/BaseWriter.hpp>
#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Writer implementation that allows to simulate data publication
 */
class MockWriter : public BaseWriter
{
public:

    //! Use parent constructors
    MockWriter(
        const types::ParticipantId& participant_id,
        const types::RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool);

    void register_write_callback(
        const std::function<void(std::unique_ptr<types::DataReceived>&)>& callback);

    /**
     * @brief Make the thread wait until message \c n has been received
     *
     * @param [in] n : wait until data number \c n has arrived and simulated to be sent
     */
    void wait_until_n_data_sent(
            uint32_t n) noexcept;

protected:

    /**
     * @brief Write specific method
     *
     * This method stores the data received in \c data_stored as if it
     * had published.
     *
     * @param data : data to simulate publication
     * @return RETCODE_OK always
     */
    utils::ReturnCode write_(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

    event::CounterWaitHandler counter_wait_handler_;

    std::function<void(std::unique_ptr<types::DataReceived>&)> callback_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_MOCKWRITER_HPP_ */

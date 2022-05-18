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
 * @file MockReader.hpp
 */

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_MOCKREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_MOCKREADER_HPP_

#include <atomic>
#include <mutex>
#include <queue>
#include <tuple>

#include <reader/implementations/auxiliar/BaseReader.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Reader implementation that allows to simulate data reception
 */
class MockReader : public BaseReader
{
public:

    //! Use parent constructors
    using BaseReader::BaseReader;

    void simulate_data_reception(
            types::Guid guid,
            void* data,
            uint32_t size) noexcept;

protected:

    /**
     * @brief Take specific method
     *
     * After \c take method, the data will be removed from \c data_to_send_ .
     *
     * @param data : oldest data to take
     * @return \c RETCODE_OK if data has been correctly taken
     * @return \c RETCODE_NO_DATA if \c data_to_send_ is empty
     */
    utils::ReturnCode take_(
            std::unique_ptr<types::DataReceived>& data) noexcept override;

    //! Stores the data that must be retrieved with \c take() method
    std::queue<std::tuple<types::Guid, void*, uint32_t>> data_to_send_;

    std::mutex mock_internal_mutex_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_MOCKREADER_HPP_ */

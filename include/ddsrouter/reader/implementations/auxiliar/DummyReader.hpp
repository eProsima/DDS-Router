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
 * @file DummyReader.hpp
 */

#ifndef _DATABROKER_READER_IMPLEMENTATIONS_AUX_DUMMYREADER_HPP_
#define _DATABROKER_READER_IMPLEMENTATIONS_AUX_DUMMYREADER_HPP_

#include <atomic>
#include <mutex>
#include <queue>

#include <ddsrouter/reader/implementations/auxiliar/BaseReader.hpp>
#include <ddsrouter/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {

//! Data that has been sent to a Dummy Reader in order to simulate data reception
struct DummyDataReceived
{
    //! Payload in a format of vector of bytes
    std::vector<PayloadUnit> payload;

    //! Guid of the source entity that has transmit the data
    Guid source_guid;
};

/**
 * Reader implementation that allows to simulate data reception
 */
class DummyReader : public BaseReader
{
public:

    //! Use parent constructors
    using BaseReader::BaseReader;

    /**
     * @brief Make the Reader to simulate data reception
     *
     * @param data : The data received (simulately)
     */
    void simulate_data_reception(
            DummyDataReceived data) noexcept;

protected:

    /**
     * @brief Take specific method
     *
     * After take method, the data will be removed from \c data_to_send_ .
     *
     * @param data : oldest data to take
     * @return RETCODE_OK if data has been correctly taken
     * @return RETCODE_NO_DATA if \c data_to_send_ is empty
     */
    ReturnCode take_(
            std::unique_ptr<DataReceived>& data) noexcept override;

    //! Stores the data that must be retrieved with \c take() method
    std::queue<DummyDataReceived> data_to_send_;

    //! Guard access to \c data_to_send_
    mutable std::recursive_mutex dummy_mutex_;
};

} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DATABROKER_READER_IMPLEMENTATIONS_AUX_DUMMYREADER_HPP_ */

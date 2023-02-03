// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#pragma once

#include <queue>

#include <ddsrouter_core/participants/reader/auxiliar/BaseReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

class MockReader
{
    virtual ~MockReader();
};

template <typename T>
class MockReaderSpecialization : public BaseReader
{
public:

    //! Use parent constructors
    using BaseReader::BaseReader;

    /**
     * @brief Simulate data reception on Reader
     *
     * @param data : The data received (by simulation)
     */
    void simulate_data_reception(
            T data) noexcept;

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
    virtual utils::ReturnCode take_(
            std::unique_ptr<core::types::IRoutingData>& data) noexcept override = 0;

    //! Stores the data that must be retrieved with \c take() method
    std::queue<T> data_to_send_;

    mutable std::mutex mutex_;
};

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */

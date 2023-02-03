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
 * @file MockWriter.hpp
 */

#pragma once

#include <cpp_utils/time/time_utils.hpp>

#include <ddsrouter_core/participants/writer/auxiliar/BaseWriter.hpp>
#include <ddsrouter_core/interface/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

class MockWriter
{
    virtual ~MockWriter();
};

template <typename T>
class MockWriterSpecialization : public BaseWriter
{
public:

    //! Use parent constructors
    using BaseWriter::BaseWriter;

    /**
     * @brief Get the data that should have been sent by this writer
     *
     * @return vector of data
     */
    std::vector<T> get_data_that_should_have_been_sent() const noexcept;


    /**
     * @brief Make the thread wait until message \c n has been received
     *
     * @param [in] n : wait until data number \c n has arrived and simulated to be sent
     */
    unsigned int count_data_received() const noexcept;

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
    virtual utils::ReturnCode write_(
            IRoutingData& data) noexcept override = 0;

    //! Stores the data that should have been published
    std::vector<T> data_stored_;

    mutable std::mutex mutex_;
};

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_DUMMYWRITER_HPP_ */

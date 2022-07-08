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
 * @file DummyWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_DUMMYWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_DUMMYWRITER_HPP_

#include <condition_variable>
#include <mutex>

#include <ddsrouter_utils/time/time_utils.hpp>

#include <writer/implementations/auxiliar/BaseWriter.hpp>
#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

//! Data kind that a Dummy Writer should have sent

struct DummyDataStored
{
    //! Payload in a format of vector of bytes
    std::vector<types::PayloadUnit> payload;

    //! Guid of the source entity that has transmitted the data
    types::Guid source_guid;

    //! Timestamp of the theoretic publication time
    utils::Timestamp timestamp;
};

/**
 * Writer implementation that allows to simulate data publication
 */
class DummyWriter : public BaseWriter
{
public:

    //! Use parent constructors
    using BaseWriter::BaseWriter;

    /**
     * @brief Get the data that should have been sent by this writer
     *
     * @return vector of data
     */
    std::vector<DummyDataStored> get_data_that_should_have_been_sent() const noexcept;


    /**
     * @brief Make the thread wait until message \c n has been received
     *
     * @param [in] n : wait until data number \c n has arrived and simulated to be sent
     */
    void wait_until_n_data_sent(
            uint16_t n) const noexcept;

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
        fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept override;

    //! Stores the data that should have been published
    std::vector<DummyDataStored> data_stored_;

    /**
     * Condition variable to wait for new data available or track termination.
     */
    mutable std::condition_variable wait_condition_variable_;

    //! Guard access to \c data_stored_
    mutable std::mutex dummy_mutex_;
};

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_DUMMYWRITER_HPP_ */

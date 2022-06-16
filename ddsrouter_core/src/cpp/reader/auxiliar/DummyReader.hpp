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

#ifndef __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_DUMMYREADER_HPP_
#define __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_DUMMYREADER_HPP_

#include <atomic>
#include <mutex>
#include <queue>

#include <reader/auxiliar/GenericReader.hpp>
#include <reader/IReader.hpp>

#include <ddsrouter_core/types/participant/ParticipantId.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Reader implementation that allows to simulate data reception
 */
template <>
class GenericReader<types::ParticipantKind::dummy> : public IReader
{
public:

    //! Use parent constructors
    using IReader::IReader;

    /**
     * @brief Take received messages and forward to readers
     *
     */
    void take_and_forward() noexcept override;

    void initialize(
            uint32_t guid_idx);

    /**
     * @brief Simulate data reception on Reader
     *
     * @param message : The data received (by simulation)
     */
    void mock_data_reception(
            const std::string& message);

    //! Return number of enqueued messages
    unsigned long get_enqueued() const;

    //! Getter for notified_
    unsigned long get_notified() const;

    //! Getter for forwarded_
    unsigned long get_forwarded() const;

protected:

    fastrtps::rtps::GUID_t mock_guid_;

    mutable std::mutex history_mutex_;

    std::queue<std::string> enqueued_messages_;

    std::atomic<unsigned long> notified_;

    unsigned long forwarded_;

    std::atomic_flag take_lock_;

};

using DummyReader = GenericReader<types::ParticipantKind::dummy>;

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_READER_IMPLEMENTATIONS_AUXILIAR_DUMMYREADER_HPP_ */

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

#include <writer/auxiliar/GenericWriter.hpp>
#include <writer/IWriter.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Writer implementation that allows to simulate data publication
 */
template <>
class GenericWriter<types::ParticipantKind::dummy> : public IWriter
{
public:

    GenericWriter(
            const types::ParticipantId& id,
            const types::RealTopic& topic,
            fastrtps::rtps::IPayloadPool* payload_pool);

    void write(
            fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept override;

    unsigned int get_received() const;

    bool check_content(
            const std::string& reference_message) const;

private:

    mutable std::mutex mutex_;

    fastrtps::rtps::GUID_t guid_;

    std::vector<std::string> all_received_;

};

using DummyWriter = GenericWriter<types::ParticipantKind::dummy>;

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_DUMMYWRITER_HPP_ */

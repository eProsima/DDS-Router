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
 * @file IWriter.hpp
 */

#ifndef __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_BASEWRITER_HPP_
#define __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_BASEWRITER_HPP_

#include <atomic>
#include <mutex>

#include <ddsrouter_core/types/endpoint/BaseWriterReader.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>

namespace eprosima {
namespace fastrtps {
namespace rtps {

class IPayloadPool;
struct CacheChange_t;

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */

namespace eprosima {
namespace ddsrouter {
namespace core {

/**
 * Base Writer that implements for other Writer specializations
 *
 * In order to inherit from this class:
 * Implement public write() method in derived specializations.
 */
class IWriter : public types::BaseWriterReader<types::EndpointKind::writer>
{
public:

    /**
     * @brief Construct a new Base Writer object
     *
     * @param participant ID
     * @param topic topic that this Writer will refer to
     * @param payload_pool DDS Router shared IPayloadPool
     */
    IWriter(
            const types::ParticipantId& id,
            const types::RealTopic& topic,
            fastrtps::rtps::IPayloadPool* payload_pool);

    virtual ~IWriter();

    /**
     * @brief Abstract interface for forwarding a cache change into a writer history
     *
     * @param reader_cache_change Reader cache change
     */
    virtual void write(
            fastrtps::rtps::CacheChange_t* reader_cache_change) noexcept = 0;

protected:

    //! Reference to Payload Pool, always outlive readers/writers in DDSRouter
    fastrtps::rtps::IPayloadPool* payload_pool_;
};

/**
 * @brief \c IWriter to stream serialization
 *
 * This method is merely a to_string of a IWriter definition.
 * It serialize the ParticipantId and topic
 */
std::ostream& operator <<(
        std::ostream& os,
        const IWriter& writer);

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_WRITER_IMPLEMENTATIONS_AUXILIAR_BASEWRITER_HPP_ */

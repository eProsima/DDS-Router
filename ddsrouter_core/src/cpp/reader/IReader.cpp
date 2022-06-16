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
 * @file IReader.cpp
 */

#include <ddsrouter_utils/Log.hpp>
#include <reader/IReader.hpp>
#include <writer/IWriter.hpp>
#include <communication/DataForwardQueue.hpp>
#include <communication/payload_pool/TopicPayloadPool.hpp>
#include <ddsrouter_core/types/endpoint/BaseWriterReader.ipp>
#include <ddsrouter_core/types/participant/ParticipantId.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

IReader::IReader(
        const ParticipantId& id,
        const RealTopic& topic,
        fastrtps::rtps::IPayloadPool* payload_pool,
        DataForwardQueue& data_forward_queue)
    : BaseWriterReader<EndpointKind::reader>(id, topic)
    , payload_pool_(payload_pool)
    , data_forward_queue_(data_forward_queue)
    , enabled_(false)
{
    logDebug(DDSROUTER_BASEREADER, "Creating Reader " << *this << ".");
}

IReader::~IReader()
{
    logDebug(DDSROUTER_BASEREADER, "Destroying Reader " << *this << ".");
}

utils::ReturnCode IReader::enable() noexcept
{
    bool expected = false;
    if (enabled_.compare_exchange_strong(expected, true, std::memory_order_release, std::memory_order_relaxed))
    {

        // Call specific enable
        enable_();

        return utils::ReturnCode::RETCODE_OK;
    }
    else
    {

        // Already enabled, do nothing
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

utils::ReturnCode IReader::disable() noexcept
{
    bool expected = true;
    if (enabled_.compare_exchange_strong(expected, false, std::memory_order_release, std::memory_order_relaxed))
    {

        // Call specific disable
        disable_();

        return utils::ReturnCode::RETCODE_OK;

    }
    else
    {

        // Already disabled, do nothing
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

void IReader::register_writer(
        IWriter* writer)
{
    if (writer->id() != this->id())
    {
        if (std::find_if(std::begin(writers_), std::end(writers_), [writer](auto wrt)
                {
                    return writer->id() == wrt->id();
                }) != std::end(writers_))
        {
            throw utils::InconsistencyException(utils::Formatter() << "Writer already introduced in this reader");
        }
        return writers_.push_back(writer);
    }
}

void IReader::enable_() noexcept
{
    // It does nothing. Optionally override this method so it has functionality.
}

void IReader::disable_() noexcept
{
    // It does nothing. Optionally override this method so it has functionality.
}

std::ostream& operator <<(
        std::ostream& os,
        const IReader& reader)
{
    os << "Reader{" << reader.id().name() << ";" << reader.topic() << "}";
    return os;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

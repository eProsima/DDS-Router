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


#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/common/CacheChange.h>

#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_participants/efficiency/cache_change/CacheChangePool.hpp>

#include <ddspipe_participants/writer/rtps/MultiWriter.hpp>
#include <ddspipe_participants/writer/rtps/QoSSpecificWriter.hpp>
#include <ddspipe_participants/writer/rtps/filter/RepeaterDataFilter.hpp>
#include <ddspipe_participants/writer/rtps/filter/SelfDataFilter.hpp>
#include <ddspipe_participants/types/dds/RouterCacheChange.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace rtps {

using namespace eprosima::ddspipe::core::types;

MultiWriter::MultiWriter(
        const ParticipantId& participant_id,
        const DdsTopic& topic,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant,
        const bool repeater /* = false */)
    : BaseWriter(participant_id, payload_pool)
    , rtps_participant_(rtps_participant)
    , repeater_(repeater)
    , topic_(topic)
{
    // Do nothing
}

MultiWriter::~MultiWriter()
{
    // Lock so no other operations is taking place
    std::unique_lock<WritersMapType> lock(writers_map_);

    // Disable every inside writer and kill it
    for (auto& writer : writers_map_)
    {
        writer.second->disable();
        delete writer.second;
    }

    logInfo(DDSROUTER_RTPS_WRITER, "Deleting MultiWriter created in Participant " <<
            participant_id_ << " for topic " << topic_);
}

void MultiWriter::enable_() noexcept
{
    std::shared_lock<WritersMapType> lock(writers_map_);
    for (auto& writer : writers_map_)
    {
        writer.second->enable();
    }
}

void MultiWriter::disable_() noexcept
{
    std::shared_lock<WritersMapType> lock(writers_map_);
    for (auto& writer : writers_map_)
    {
        writer.second->disable();
    }
}

bool MultiWriter::exist_partition_(
        const core::types::SpecificEndpointQoS& data_qos)
{
    std::shared_lock<WritersMapType> lock(writers_map_);
    return writers_map_.find(data_qos) != writers_map_.end();
}

QoSSpecificWriter* MultiWriter::get_writer_or_create_(
        const core::types::SpecificEndpointQoS& data_qos)
{
    // NOTE: it uses unique lock because it may change the database, and there is no way
    // to do so if taking share and unique must be done.
    std::unique_lock<WritersMapType> lock(writers_map_);

    // Get if it exists
    auto it = writers_map_.find(data_qos);
    if (it != writers_map_.end())
    {
        return it->second;
    }

    // Create Writer
    QoSSpecificWriter* new_writer = create_writer_nts_(data_qos);
    // Add it to map
    writers_map_[data_qos] = new_writer;

    // If this is enabled, enable writer
    if (enabled_)
    {
        new_writer->enable();
    }

    return new_writer;
}

QoSSpecificWriter* MultiWriter::create_writer_nts_(
        const core::types::SpecificEndpointQoS& data_qos)
{
    logDebug(
        DDSROUTER_MULTIWRITER,
        "Creating a new Writer in " << *this << " for qos " << data_qos << ".");

    auto writer = new QoSSpecificWriter(
        this->participant_id_,
        this->topic_,
        this->payload_pool_,
        this->rtps_participant_,
        data_qos,
        repeater_);
    writer->init();

    return writer;
}

// Specific enable/disable do not need to be implemented
utils::ReturnCode MultiWriter::write_(
        core::IRoutingData& data) noexcept
{
    auto& rtps_data = dynamic_cast<core::types::RtpsPayloadData&>(data);

    logDebug(
        DDSROUTER_MULTIWRITER,
        "Writing in Partitions Writer " << *this << " a data with qos " << rtps_data.writer_qos << " from " <<
            rtps_data.source_guid);

    // Take Writer
    auto this_qos_writer = get_writer_or_create_(rtps_data.writer_qos);

    logDebug(
        DDSROUTER_MULTIWRITER,
        "Writer chosen to send is " << *this_qos_writer << ".");

    // Write
    return this_qos_writer->write(data);
}

} /* namespace rtps */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */

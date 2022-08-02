// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file RPCBridge.cpp
 *
 */

#include <functional>

#include <communication/rpc/RPCBridge.hpp>

#include <ddsrouter_utils/exception/UnsupportedException.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <participant/implementations/rtps/CommonRTPSRouterParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

RPCBridge::RPCBridge(
        const RPCTopic& topic,
        std::shared_ptr<ParticipantsDatabase> participants_database,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<utils::SlotThreadPool> thread_pool,
        bool enable /* = false */)
    : Bridge(participants_database, payload_pool, thread_pool, enable)
    , topic_(topic)
{
    logDebug(DDSROUTER_RPCBRIDGE, "Creating RPCBridge " << *this << ".");

    std::set<ParticipantId> ids = participants_->get_rtps_participants_ids();

    // Create a proxy server in each RTPS participant
    for (ParticipantId id: ids)
    {
        std::shared_ptr<IParticipant> participant = participants_->get_participant(id);

        // Safe casting as we are only getting RTPS participants
        reply_writers_[id] = std::static_pointer_cast<rtps::CommonRTPSRouterParticipant<configuration::ParticipantConfiguration>>(participant)->create_reply_writer(topic);
        request_readers_[id] = std::static_pointer_cast<rtps::Reader>(participant->create_reader(topic.request_topic()));

        create_slot(request_readers_[id]);
    }

    if (enable)
    {
        this->enable();
    }

    logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this << " created.");
}

RPCBridge::~RPCBridge()
{
    logDebug(DDSROUTER_RPCBRIDGE, "Destroying RPCBridge " << *this << ".");

    // Disable all entities before destruction
    disable();

    // Remove all created Writers and Readers
    for (ParticipantId id: participants_->get_participants_ids())
    {
        std::shared_ptr<IParticipant> participant = participants_->get_participant(id);

        auto request_writer = request_writers_.find(id);
        if (request_writer != request_writers_.end())
        {
            participant->delete_writer(request_writer->second);
            request_writers_.erase(request_writer);
        }

        auto reply_writer = reply_writers_.find(id);
        if (reply_writer != reply_writers_.end())
        {
            participant->delete_writer(reply_writer->second);
            reply_writers_.erase(reply_writer);
        }

        auto request_reader = request_readers_.find(id);
        if (request_reader != request_readers_.end())
        {
            participant->delete_reader(request_reader->second);
            request_readers_.erase(request_reader);
        }

        auto reply_reader = reply_readers_.find(id);
        if (reply_reader != reply_readers_.end())
        {
            participant->delete_reader(reply_reader->second);
            reply_readers_.erase(reply_reader);
        }
    }

    // Participants must not be removed as they belong to the Participant Database

    logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this << " destroyed.");
}

void RPCBridge::enable() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!enabled_)
    {
        logInfo(DDSROUTER_RPCBRIDGE, "Enabling RPCBridge for service " << topic_ << ".");

        enabled_ = true;

        for (auto& writer_it : request_writers_)
        {
            writer_it.second->enable();
        }

        for (auto& reader_it : reply_readers_)
        {
            reader_it.second->enable();
        }

        for (auto& writer_it : reply_writers_)
        {
            writer_it.second->enable();
        }

        for (auto& reader_it : request_readers_)
        {
            reader_it.second->enable();
        }
    }
}

void RPCBridge::disable() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (enabled_)
    {
        logInfo(DDSROUTER_RPCBRIDGE, "Disabling RPCBridge for service " << topic_ << ".");

        enabled_ = false;

        {
            // Wait for a transmission to finish before disabling endpoints
            std::unique_lock<std::shared_timed_mutex> lock(on_transmission_mutex_);
        }

        for (auto& reader_it : request_readers_)
        {
            reader_it.second->disable();
        }

        for (auto& writer_it : reply_writers_)
        {
            writer_it.second->disable();
        }

        for (auto& reader_it : reply_readers_)
        {
            reader_it.second->disable();
        }

        for (auto& writer_it : request_writers_)
        {
            writer_it.second->disable();
        }
    }
}

void RPCBridge::create_service_registry(ParticipantId server_participant_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it_registry = service_registries_.find(server_participant_id);

    if (it_registry != service_registries_.end())
    {
        // Server already discovered in this participant, reuse registry
        return;
    }

    std::shared_ptr<ServiceRegistry> service_registry = std::make_shared<ServiceRegistry>(topic_, server_participant_id);

    // Create proxy client
    std::shared_ptr<IParticipant> participant = participants_->get_participant(server_participant_id);

    // Safe casting as participant discovering service can only be of RTPS kind
    request_writers_[server_participant_id] = std::static_pointer_cast<rtps::CommonRTPSRouterParticipant<configuration::ParticipantConfiguration>>(participant)->create_request_writer(topic_, service_registry);
    reply_readers_[server_participant_id] = std::static_pointer_cast<rtps::Reader>(participant->create_reader(topic_.reply_topic()));

    create_slot(reply_readers_[server_participant_id]);

    // Variable also accessed from writing thread, but thread-safe as request writer is not enabled yet
    service_registry->set_related_sample_identity_nts(reply_readers_[server_participant_id]->guid());
    service_registries_[server_participant_id] = service_registry;

    // enable proxy client's endpoints as this function is only called when the topic is allowed, and enabling the whole bridge would skip their activation if bridge already enabled
    request_writers_[server_participant_id]->enable();
    reply_readers_[server_participant_id]->enable();
}

void RPCBridge::data_available_(const Guid& reader_guid) noexcept
{
    // Only hear callback if it is enabled
    if (enabled_)
    {
        logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this << " has data ready to be sent.");

        // Protected by internal RTPS Reader mutex, as called within \c onNewCacheChangeAdded callback

        std::pair<bool, utils::TaskId>& task = tasks_map_[reader_guid];
        if (!task.first)
        {
            thread_pool_->emit(task.second);
            task.first = true;
            logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this << " send callback to queue.");
        }
        else
        {
            logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this << " callback NOT sent (task already queued).");
        }
    }
}

void RPCBridge::transmit_(std::shared_ptr<rtps::Reader> reader) noexcept
{
    // Avoid being disabled while transmitting
    std::shared_lock<std::shared_timed_mutex> lock(on_transmission_mutex_);

    while (enabled_)
    {
        // Get data received
        std::unique_ptr<DataReceived> data = std::make_unique<DataReceived>();
        utils::ReturnCode ret;
        {
            std::lock_guard<std::recursive_timed_mutex> lock(reader->get_internal_mutex());

            ret = reader->take(data);
            if (ret == utils::ReturnCode::RETCODE_NO_DATA)
            {
                // info or debuglogInfo(DDSROUTER_RPCBRIDGE, "No data found at service Reader in topic " << reader->topic()
                                                                        // << ". Finish transmission.");

                tasks_map_[reader->guid()].first = false;
                return;
            }
        }

        if (!ret)
        {
            // Error reading data
            logWarning(DDSROUTER_RPCBRIDGE, "Error taking data at service Reader in topic " << reader->topic()
                                                                        << ". Error code " << ret
                                                                      << ". Skipping data and continue.");
            continue;
        }

        // logDebug(DDSROUTER_RPCBRIDGE,
        //         "Track " << reader_participant_id_ << " for topic " << topic_ <<
        //         " transmitting data from remote endpoint " << data->source_guid << ".");

        // different logs for request and reply

        if (RPCTopic::is_request_topic(reader->topic()))
        {
            for (auto& service_registry : service_registries_)
            {
                // TODO: Include is_repeater() condition
                if (data->participant_receiver == service_registry.first || !service_registry.second->enabled())
                {
                    continue;
                }

                ret = request_writers_[service_registry.first]->write(data);

                if (!ret)
                {
                    // logWarning(DDSROUTER_RPCBRIDGE, "Error writting data in Track " << topic_ << ". Error code "
                    //                                                             << ret <<
                    //         ". Skipping data for this writer and continue.");
                    continue;
                }
            }
        }
        else if (RPCTopic::is_reply_topic(reader->topic()))
        {
            std::pair<ParticipantId, SampleIdentity> registry_entry = service_registries_[reader->participant_id()]->get(data->write_params.sample_identity().sequence_number());

            if (!registry_entry.first.is_valid())
            {
                // Request already replied by another server connected to the same participant as this one
                payload_pool_->release_payload(data->payload);
                continue;
            }

            ret = reply_writers_[registry_entry.first]->write(data, registry_entry.second);

            if (!ret)
            {
                // logWarning(DDSROUTER_RPCBRIDGE, "Error writting data in Track " << topic_ << ". Error code "
                //                                                             << ret <<
                //         ". Skipping data for this writer and continue.");
                payload_pool_->release_payload(data->payload);
                continue;
            }
            service_registries_[reader->participant_id()]->erase(data->write_params.sample_identity().sequence_number());
        }
        else
        {
            // tssnh
        }

        payload_pool_->release_payload(data->payload);
    }
}

void RPCBridge::create_slot(std::shared_ptr<rtps::Reader> reader) noexcept
{
    Guid reader_guid = reader->guid();

    reader->set_on_data_available_callback(
                [=]() { data_available_(reader_guid); });

    // Set slot in thread pool for this reader
    utils::TaskId task_id = utils::new_unique_task_id();
    thread_pool_->slot(
            task_id,
            [=]() { transmit_(reader); });
    tasks_map_[reader_guid] = {false, task_id};
}

std::ostream& operator <<(
        std::ostream& os,
        const RPCBridge& bridge)
{
    os << "RPCBridge{" << bridge.topic_ << "}";
    return os;
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

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

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/utils.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

RPCBridge::RPCBridge(
        const RPCTopic& topic,
        std::shared_ptr<ParticipantsDatabase> participants_database,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<utils::SlotThreadPool> thread_pool)
    : Bridge(participants_database, payload_pool, thread_pool)
    , topic_(topic)
    , init_(false)
{
    logDebug(DDSROUTER_RPCBRIDGE, "Creating RPCBridge " << *this << ".");

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

void RPCBridge::init_nts_()
{
    logInfo(DDSROUTER_RPCBRIDGE, "Creating endpoints in RPCBridge for service " << topic_ << ".");

    std::set<ParticipantId> ids = participants_->get_rtps_participants_ids();

    // Create a proxy client and server in each RTPS participant
    for (ParticipantId id: ids)
    {
        create_proxy_client_nts_(id);
        create_proxy_server_nts_(id);
        if (current_servers_[id].size())
        {
            service_registries_[id]->enable();
        }
    }

    // TODO: This should not be done
    // Wait for the new entities created to match before sending data from one side to the other
    // utils::sleep_for(500);

    init_ = true;
}

void RPCBridge::create_proxy_server_nts_(
        ParticipantId participant_id)
{
    std::shared_ptr<IParticipant> participant = participants_->get_participant(participant_id);

    // Safe casting as we are only getting RTPS participants
    reply_writers_[participant_id] =
            std::static_pointer_cast<rtps::Writer>(participant->create_writer(topic_.reply_topic()));
    request_readers_[participant_id] =
            std::static_pointer_cast<rtps::Reader>(participant->create_reader(topic_.request_topic()));

    create_slot_(request_readers_[participant_id]);
}

void RPCBridge::create_proxy_client_nts_(
        ParticipantId participant_id)
{
    std::shared_ptr<IParticipant> participant = participants_->get_participant(participant_id);

    // Safe casting as we are only getting RTPS participants
    request_writers_[participant_id] =
            std::static_pointer_cast<rtps::Writer>(participant->create_writer(topic_.request_topic()));
    reply_readers_[participant_id] =
            std::static_pointer_cast<rtps::Reader>(participant->create_reader(topic_.reply_topic()));

    create_slot_(reply_readers_[participant_id]);

    // Create service registry associated to this proxy client
    SampleIdentity related_sample_identity;
    related_sample_identity.writer_guid(reply_readers_[participant_id]->guid());
    service_registries_[participant_id] = std::make_shared<ServiceRegistry>(topic_, participant_id,
                    related_sample_identity);
}

void RPCBridge::enable() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!enabled_ && servers_available_())
    {
        logInfo(DDSROUTER_RPCBRIDGE, "Enabling RPCBridge for service " << topic_ << ".");

        if (!init_)
        {
            try
            {
                init_nts_();
            }
            catch (const utils::InitializationException& e)
            {
                logError(DDSROUTER_RPCBRIDGE,
                        "Error while creating endpoints in RPCBridge for service " << topic_ <<
                        ". Error code:" << e.what() << ".");
                return;
            }
        }

        enabled_ = true;

        // Enable writers before readers, to avoid starting a transmission (not protected with \c mutex_) which may
        // attempt to write with a yet disabled writer

        for (auto& writer_it : request_writers_)
        {
            writer_it.second->enable();
        }

        for (auto& writer_it : reply_writers_)
        {
            writer_it.second->enable();
        }

        for (auto& reader_it : reply_readers_)
        {
            reader_it.second->enable();
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

        for (auto& reader_it : reply_readers_)
        {
            reader_it.second->disable();
        }

        for (auto& writer_it : reply_writers_)
        {
            writer_it.second->disable();
        }

        for (auto& writer_it : request_writers_)
        {
            writer_it.second->disable();
        }
    }
}

void RPCBridge::discovered_service(
        const types::ParticipantId& server_participant_id,
        const types::GuidPrefix& server_guid_prefix) noexcept
{
    current_servers_[server_participant_id].emplace(server_guid_prefix);
    if (init_)
    {
        service_registries_[server_participant_id]->enable();
    }
}

void RPCBridge::removed_service(
        const types::ParticipantId& server_participant_id,
        const types::GuidPrefix& server_guid_prefix) noexcept
{
    current_servers_[server_participant_id].erase(server_guid_prefix);
    if (!current_servers_[server_participant_id].size() && !servers_available_())
    {
        disable();
    }
}

bool RPCBridge::servers_available_() const noexcept
{
    for (auto it = current_servers_.begin(); it != current_servers_.end(); it++)
    {
        if (it->second.size())
        {
            return true;
        }
    }
    return false;
}

void RPCBridge::data_available_(
        const Guid& reader_guid) noexcept
{
    // Only hear callback if it is enabled
    if (enabled_)
    {
        logDebug(
            DDSROUTER_RPCBRIDGE, "RPCBridge " << *this <<
                                 " has data ready to be sent in reader " << reader_guid << " .");

        // Protected by internal RTPS Reader mutex, as called within \c onNewCacheChangeAdded callback

        std::pair<bool, utils::TaskId>& task = tasks_map_[reader_guid];
        if (!task.first)
        {
            task.first = true;
            thread_pool_->emit(task.second);
            logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this <<
                " - " << reader_guid << " send callback to queue.");
        }
        else
        {
            logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this <<
                " - " << reader_guid << " callback NOT sent (task already queued).");
        }
    }
}

void RPCBridge::transmit_(
        std::shared_ptr<rtps::Reader> reader) noexcept
{
    // Avoid being disabled while transmitting
    std::shared_lock<std::shared_timed_mutex> lock(on_transmission_mutex_);

    logDebug(DDSROUTER_RPCBRIDGE, "RPCBridge " << *this <<
        " transmitting for reader " << reader->guid() << " .");

    while (enabled_)
    {
        {
            std::lock_guard<eprosima::fastrtps::RecursiveTimedMutex> lock(reader->get_internal_mutex());

            if (!(reader->get_unread_count() > 0))
            {
                // No more data to be read -> finish transmission
                tasks_map_[reader->guid()].first = false;

                logDebug(DDSROUTER_RPCBRIDGE,
                    "RPCBridge service " << *this << " finishing transmitting because no more data available.");

                return;
            }
        }

        // Get data received
        std::unique_ptr<DataReceived> data = std::make_unique<DataReceived>();
        utils::ReturnCode ret = reader->take(data);

        // Will never return \c RETCODE_NO_DATA, otherwise would have finished before
        if (!ret)
        {
            // Error reading data
            logWarning(DDSROUTER_RPCBRIDGE,
                    "Error taking data at service Reader in topic " << reader->topic()
                                                                    << ". Error code " << ret
                                                                    << ". Skipping data and continue.");
            continue;
        }

        if (RPCTopic::is_request_topic(reader->topic()))
        {
            logDebug(DDSROUTER_RPCBRIDGE,
                    "RPCBridge for service " << topic_ <<
                    " transmitting request from remote endpoint " << data->source_guid << ".");

            SampleIdentity reply_related_sample_identity = data->write_params.sample_identity();
            reply_related_sample_identity.sequence_number(data->sequenceNumber);
            if (reply_related_sample_identity == SampleIdentity::unknown())
            {
                logWarning(DDSROUTER_RPCBRIDGE,
                        "RPCBridge for service " << topic_ <<
                        " received ill-formed request from remote endpoint " << data->source_guid << ". Ignoring...");
                break;
            }

            for (auto& service_registry : service_registries_)
            {
                // Do not send request through same participant who received it (unless repeater), or if there are no servers to process it
                if ((data->participant_receiver == service_registry.first &&
                        !participants_->get_participant(service_registry.first)->is_repeater()) ||
                        !service_registry.second->enabled())
                {
                    continue;
                }

                eprosima::fastrtps::rtps::WriteParams wparams;
                eprosima::fastrtps::rtps::SequenceNumber_t sequence_number;

                // Attach the information the server needs in order to reply to the appropiate proxy client.
                // Thread-safe as registry's related_sample_identity is set only in creation, when this writer is still disabled.
                // Also note that concurrent \c write_ operations in the same \c RequestWriter are not possible, as \c write from parent \c BaseWriter is guarded.
                wparams.related_sample_identity(service_registry.second->related_sample_identity_nts());

                ret = request_writers_[service_registry.first]->write(data, wparams, sequence_number);

                if (!ret)
                {
                    logWarning(DDSROUTER_RPCBRIDGE, "Error writting request in RPCBridge for service "
                            << topic_ << ". Error code " << ret << ". Skipping data for this writer and continue.");
                    continue;
                }

                // Add entry to registry associated to the transmission of this request through this proxy client.
                service_registry.second->add(sequence_number, {data->participant_receiver,
                                                               reply_related_sample_identity});
            }
        }
        else if (RPCTopic::is_reply_topic(reader->topic()))
        {
            logDebug(DDSROUTER_RPCBRIDGE,
                    "RPCBridge for service " << topic_ <<
                    " transmitting reply from remote endpoint " << data->source_guid << ".");

            // A Server could be answering a different client in this same DDS Router or a remote client
            // Thus, it must be filtered so only replies to this client are processed.
            if (data->write_params.sample_identity().writer_guid() != reader->guid())
            {
                logDebug(DDSROUTER_RPCBRIDGE,
                        "RPCBridge for service " << *this << " from reader " << reader->guid() <<
                        " received response meant for other client: " <<
                        data->write_params.sample_identity().writer_guid());
            }
            else
            {
                // Fetch information required for transmission; which proxy server should send it and with what parameters
                std::pair<ParticipantId,
                        SampleIdentity> registry_entry = service_registries_[reader->participant_id()]->get(
                    data->write_params.sample_identity().sequence_number());

                // Not valid means:
                //   Case 1: (SimpleParticipant) Request already replied by another server connected to the same participant as this one.
                //   Case 2: (WAN Participant repeater) Request already replied by another PROXY server connected to the same participant as this one.
                //   Case 3: (SimpleParticipant) Received reply from a server in same domain as client, thus the request was ignored and no entry was added.
                if (registry_entry.first.is_valid())
                {
                    eprosima::fastrtps::rtps::WriteParams wparams;
                    wparams.related_sample_identity(registry_entry.second);
                    ret = reply_writers_[registry_entry.first]->write(data, wparams);

                    if (!ret)
                    {
                        logWarning(DDSROUTER_RPCBRIDGE, "Error writting reply in RPCBridge for service "
                                << topic_ << ". Error code " << ret << ".");
                    }
                    else
                    {
                        service_registries_[reader->participant_id()]->erase(
                            data->write_params.sample_identity().sequence_number());
                    }
                }
            }
        }
        else
        {
            utils::tsnh(
                utils::Formatter() << "Data to be transmitted in RPCBridge is not in RPCTopic.");
        }

        payload_pool_->release_payload(data->payload);
    }

    logDebug(DDSROUTER_RPCBRIDGE,
        "RPCBridge service " << *this << " finishing transmitting.");

    // No need to take internal mutex as bridge is disabled (and cannot be enabled until \c disable releases bridge
    // mutex, which cannot occur until this thread releases transmission mutex), so \c data_available_ won't take effect
    tasks_map_[reader->guid()].first = false;
}

void RPCBridge::create_slot_(
        std::shared_ptr<rtps::Reader> reader) noexcept
{
    Guid reader_guid = reader->guid();

    reader->set_on_data_available_callback(
        [=]()
        {
            data_available_(reader_guid);
        });

    // Set slot in thread pool for this reader
    utils::TaskId task_id = utils::new_unique_task_id();
    thread_pool_->slot(
        task_id,
        [=]()
        {
            transmit_(reader);
        });
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

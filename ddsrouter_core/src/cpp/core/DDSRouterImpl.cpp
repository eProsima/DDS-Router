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
 * @file DDSRouterImpl.cpp
 *
 */

#include <core/DDSRouterImpl.hpp>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantId.ipp>
#include <ddsrouter_core/types/endpoint/BaseWriterReader.ipp>


namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

DDSRouterImpl::DDSRouterImpl(
        const configuration::DDSRouterConfiguration& configuration)
    : router_configuration_(configuration)
    , workers_thread_pool_(data_forward_queue_, configuration.threads())
    , router_enabled_(false)
{
    logDebug(DDSROUTER, "Creating DDS Router.");

    // Add callback to be called by the discovery database when an Endpoint is discovered
    discovery_database_.add_endpoint_discovered_callback(std::bind(&DDSRouterImpl::discovered_endpoint_, this,
            std::placeholders::_1));

    // Load Participants
    init_participants_();

    for (const auto& topic : router_configuration_.builtin_topics())
    {
        this->register_topic_(topic);
    }

    // Init discovery database
    // The entities should not be added to the Discovery Database until the builtin topics have been created.
    // This is due to the fact that the Participants endpoints start discovering topics with different configuration
    // than the one specified in the yaml configuration file.
    discovery_database_.enable();


    logDebug(DDSROUTER, "DDS Router created.");
}

DDSRouterImpl::~DDSRouterImpl()
{
    logDebug(DDSROUTER, "Destroying DDS Router.");

    // Stop all communications
    stop();

    // Destroy Participants
    for (auto participant : participants_iterable_)
    {
        auto owned_participant = participants_registry_.pop_participant(participant->id().name());

        if (!participant)
        {
            logDevError(DDSROUTER, "Error poping participant from database.");
        }

        owned_participant.reset(); // Destructor invoked
    }

    logDebug(DDSROUTER, "DDS Router destroyed.");
}

utils::ReturnCode DDSRouterImpl::reload_configuration(
        const configuration::DDSRouterReloadConfiguration& new_configuration)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    utils::ReturnCode return_code = utils::ReturnCode::RETCODE_NO_DATA;

    auto new_builtin_topics = router_configuration_.reload(new_configuration);

    // 1. Register all new topics
    for (const auto& new_topic : new_builtin_topics)
    {
        this->register_topic_(new_topic);

        return_code = utils::ReturnCode::RETCODE_OK;
    }

    // 2. Allow and block lists may have changed, so enable/disable topics accordingly
    // No effects will happen if already enabled/disabled
    for (const auto& topic : router_configuration_.builtin_topics())
    {
        if (router_configuration_.is_topic_allowed(topic))
        {

            // Topic allowed:

            if (this->enable_topic_(topic) == utils::ReturnCode::RETCODE_OK)
            {

                logDebug(DDSROUTER, "Enabled previously disabled topic: " << topic << ".");

                return_code = utils::ReturnCode::RETCODE_OK;

            }
            else
            {

                logDebug(DDSROUTER, "Topic was already enabled: " << topic << ".");
            }

        }
        else
        {

            // Topic not allowed

            if (this->disable_topic_(topic) == utils::ReturnCode::RETCODE_OK)
            {

                logDebug(DDSROUTER, "Disabled previously enabled topic: " << topic << ".");

                return_code = utils::ReturnCode::RETCODE_OK;

            }
            else
            {

                logDebug(DDSROUTER, "Topic was already disabled: " << topic << ".");
            }
        }
    }

    return return_code;
}

utils::ReturnCode DDSRouterImpl::start() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!router_enabled_.load())
    {
        workers_thread_pool_.start_workers();

        logInfo(DDSROUTER, "Starting DDS Router.");

        enable_all_topics_();

        router_enabled_.store(true);

        return utils::ReturnCode::RETCODE_OK;
    }
    else
    {
        logInfo(DDSROUTER, "Trying to start an already enabled DDS Router.");
        return utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
}

utils::ReturnCode DDSRouterImpl::stop() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (router_enabled_.load())
    {
        logInfo(DDSROUTER, "Stopping DDS Router.");

        disable_all_topics_();

        workers_thread_pool_.stop_workers();

        router_enabled_.store(false);

        return utils::ReturnCode::RETCODE_OK;
    }
    else
    {
        logInfo(DDSROUTER, "Trying to stop a disabled DDS Router.");
        return utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
}

void DDSRouterImpl::init_participants_()
{
    // If DDS Router has not two or more Participants configured, it should fail

    auto participants_configurations = router_configuration_.participants_configurations();

    for (const auto& participant_config : participants_configurations)
    {
        // Create participant
        // This should not be in try catch case as if it fails the whole init must fail

        participants_iterable_.insert( participants_registry_.add_participant( *participant_config,
                discovery_database_));
    }
}

void DDSRouterImpl::register_topic_(
        const RealTopic& topic)
{

    // Must be first registered in the configuration
    if (!router_configuration_.is_topic_registered(topic))
    {
        throw utils::InconsistencyException("Cannot call register_topic_ before registration into configuration");
    }

    logDebug(DDSROUTER, "Registering topic into participants: " << topic << ".");

    std::lock_guard<std::recursive_mutex> lck(mutex_);

    // 1. Create (if not created before) payload pool for this topic
    auto payload_pool = this->payload_pool_registry_.get(
        router_configuration_.get_payload_pool_index(topic.name()),
        router_configuration_.payload_pool_configuration());

    if (!payload_pool)
    {
        throw utils::InconsistencyException("Payload pool registry returned null pool");
    }

    if (payload_pool->payload_pool_allocated_size() == 0)
    {
        payload_pool->reserve_history(router_configuration_.payload_pool_configuration(), false);
    }

    // 2. Create all writers and readers for this topic, iterating over all participants
    logDebug(DDSROUTER, "Creating all writers of " << topic << " for all participants");

    std::vector<IWriter*> topic_writers;
    std::vector<IReader*> topic_readers;

    for (auto participant : participants_iterable_)
    {

        auto writer_reader_pair = participant->register_topic(topic, payload_pool, data_forward_queue_);

        topic_writers.push_back( writer_reader_pair.first );
        topic_readers.push_back( writer_reader_pair.second );
    }


    // 3. Bind all distinct writers and readers for this topic
    logDebug(DDSROUTER, "Binding writers and readers for topic " << topic);

    for (auto writer : topic_writers)
    {
        for (auto reader : topic_readers)
        {
            if (writer->id() != reader->id())
            {
                reader->register_writer(writer);
            }
        }
    }

    // 4. If Router is enabled and topic allowed, enable it
    if (router_enabled_.load() && router_configuration_.is_topic_allowed(topic))
    {
        this->enable_topic_(topic);
    }

}

void DDSRouterImpl::discovered_endpoint_(
        const Endpoint& endpoint) noexcept
{
    logDebug(DDSROUTER, "Endpoint discovered in DDS Router core: " << endpoint << ".");

    // Following actions can be called by an uncertain thread so protect
    std::lock_guard<std::recursive_mutex> lck(mutex_);

    if (this->router_configuration_.register_topic(endpoint.topic()))
    {
        // Topic inserted

        logDebug(DDSROUTER, "New topic registered: " << endpoint.topic());

        this->register_topic_(endpoint.topic());

    }
    else
    {
        logDebug(DDSROUTER, "Topic already registered, skipped register: " << endpoint.topic());
    }
}

utils::ReturnCode DDSRouterImpl::enable_topic_(
        const RealTopic& topic) noexcept
{
    utils::ReturnCode return_code = utils::ReturnCode::RETCODE_NOT_ENABLED;

    logDebug(DDSROUTER, "Sending enable topic to participants: " << topic);

    for (auto participant : participants_iterable_)
    {
        if (participant->enable_topic(topic) == utils::ReturnCode::RETCODE_OK)
        {
            return_code = utils::ReturnCode::RETCODE_OK;
        }
    }

    logDebug(DDSROUTER, "Any enabled ? " << return_code);

    return return_code;
}

utils::ReturnCode DDSRouterImpl::disable_topic_(
        const RealTopic& topic) noexcept
{
    utils::ReturnCode return_code = utils::ReturnCode::RETCODE_NOT_ENABLED;

    logDebug(DDSROUTER, "Sending disable topic to participants: " << topic);

    for (auto participant : participants_iterable_)
    {
        if (participant->disable_topic(topic) == utils::ReturnCode::RETCODE_OK)
        {
            return_code = utils::ReturnCode::RETCODE_OK;
        }
    }

    logDebug(DDSROUTER, "Any disabled ? " << return_code);

    return return_code;
}

void DDSRouterImpl::enable_all_topics_() noexcept
{
    for (const auto& topic : router_configuration_.builtin_topics())
    {
        if (router_configuration_.is_topic_allowed(topic))
        {
            this->enable_topic_(topic);
        }
    }
}

void DDSRouterImpl::disable_all_topics_() noexcept
{
    for (const auto& topic : router_configuration_.builtin_topics())
    {
        this->disable_topic_(topic);
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

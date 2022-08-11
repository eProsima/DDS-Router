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

#include <set>

#include <ddsrouter_utils/exception/UnsupportedException.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/Log.hpp>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>

#include <core/DDSRouterImpl.hpp>
#include <efficiency/payload/FastPayloadPool.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

// TODO: Use initial topics to start execution and start bridges

DDSRouterImpl::DDSRouterImpl(
        const configuration::DDSRouterConfiguration& configuration)
    : payload_pool_(new FastPayloadPool())
    , participants_database_(new ParticipantsDatabase())
    , discovery_database_(new DiscoveryDatabase())
    , configuration_(configuration)
    , enabled_(false)
    , thread_pool_(std::make_shared<utils::SlotThreadPool>(configuration_.number_of_threads))
{
    logDebug(DDSROUTER, "Creating DDS Router.");

    // Check that the configuration is correct
    utils::Formatter error_msg;
    if (!configuration_.is_valid(error_msg))
    {
        throw utils::ConfigurationException(
                  utils::Formatter() <<
                      "Configuration for DDS Router is invalid: " << error_msg);
    }

    // Add callback to be called by the discovery database when an Endpoint is discovered
    discovery_database_->add_endpoint_discovered_callback(std::bind(&DDSRouterImpl::discovered_endpoint_, this,
            std::placeholders::_1));

    // Init topic allowed
    init_allowed_topics_();
    // Load Participants
    init_participants_();
    // Create Bridges
    init_bridges_();
    // Init discovery database
    // The entities should not be added to the Discovery Database until the builtin topics have been created.
    // This is due to the fact that the Participants endpoints start discovering topics with different configuration
    // than the one specified in the yaml configuration file.
    discovery_database_->enable();


    logDebug(DDSROUTER, "DDS Router created.");
}

DDSRouterImpl::~DDSRouterImpl()
{
    logDebug(DDSROUTER, "Destroying DDS Router.");

    // Stop all communications
    stop_();

    // Destroy Bridges, so Writers and Readers are destroyed before the Databases
    bridges_.clear();

    // Destroy Participants
    while (!participants_database_->empty())
    {
        auto participant = participants_database_->pop_();

        if (!participant)
        {
            logDevError(DDSROUTER, "Error poping participant from database.");
        }
        else
        {
            participant_factory_.remove_participant(participant);
        }
    }

    // There is no need to destroy shared ptrs as they will delete itslefs with 0 references

    logDebug(DDSROUTER, "DDS Router destroyed.");
}

utils::ReturnCode DDSRouterImpl::reload_configuration(
        const configuration::DDSRouterReloadConfiguration& new_configuration)
{
    // Check that the configuration is correct
    utils::Formatter error_msg;
    if (!new_configuration.is_valid(error_msg))
    {
        throw utils::ConfigurationException(
                  utils::Formatter() <<
                      "Configuration for Reload DDS Router is invalid: " << error_msg);
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_.load())
    {
        logDebug(DDSROUTER, "Reloading DDS Router configuration...");

        // Load new configuration and check it is okey
        AllowedTopicList new_allowed_topic_list(
            new_configuration.allowlist,
            new_configuration.blocklist);

        // Check if there are any new builtin topics
        std::set<RealTopic> new_builtin_topics;
        for (auto builtin_topic : new_configuration.builtin_topics)
        {
            if (current_topics_.find(*builtin_topic) == current_topics_.end())
            {
                new_builtin_topics.insert(*builtin_topic);
            }
        }

        // Check if it should change or is the same configuration
        if (new_allowed_topic_list == allowed_topics_ && new_builtin_topics.empty())
        {
            logDebug(DDSROUTER, "Same configuration, do nothing in reload.");
            return utils::ReturnCode::RETCODE_NO_DATA;
        }

        // Set new Allowed list
        allowed_topics_ = new_allowed_topic_list;

        logDebug(DDSROUTER, "New DDS Router allowed topics configuration: " << allowed_topics_);

        // It must change the configuration. Check every topic discovered and active if needed.
        for (auto& topic_it : current_topics_)
        {
            // If topic is active and it is blocked, deactivate it
            if (topic_it.second)
            {
                if (!allowed_topics_.is_topic_allowed(topic_it.first))
                {
                    deactivate_topic_(topic_it.first);
                }
            }
            else
            {
                // If topic is not active and it is allowed, activate it
                if (allowed_topics_.is_topic_allowed(topic_it.first))
                {
                    activate_topic_(topic_it.first);
                }
            }
        }

        // Create bridges for newly added builtin topics
        for (RealTopic topic : new_builtin_topics)
        {
            discovered_topic_(topic);
        }

        configuration_.reload(new_configuration);

        return utils::ReturnCode::RETCODE_OK;
    }
    else
    {
        return utils::ReturnCode::RETCODE_NOT_ENABLED;
    }
}

utils::ReturnCode DDSRouterImpl::start() noexcept
{
    utils::ReturnCode ret = start_();
    if (ret == utils::ReturnCode::RETCODE_OK)
    {
        logInfo(DDSROUTER, "Starting DDS Router.");
    }
    else if (ret == utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET)
    {
        logInfo(DDSROUTER, "Trying to start an enabled DDS Router.");
    }

    return ret;
}

utils::ReturnCode DDSRouterImpl::stop() noexcept
{
    utils::ReturnCode ret = stop_();
    if (ret == utils::ReturnCode::RETCODE_OK)
    {
        logInfo(DDSROUTER, "Stopping DDS Router.");
    }
    else if (ret == utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET)
    {
        logInfo(DDSROUTER, "Trying to stop a not enabled DDS Router.");
    }

    return ret;
}

utils::ReturnCode DDSRouterImpl::start_() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!enabled_.load())
    {
        enabled_.store(true);

        logInfo(DDSROUTER, "Starting DDS Router.");

        // Enable thread pool
        thread_pool_->enable();

        activate_all_topics_();
        return utils::ReturnCode::RETCODE_OK;
    }
    else
    {
        logInfo(DDSROUTER, "Trying to start an already enabled DDS Router.");
        return utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
}

utils::ReturnCode DDSRouterImpl::stop_() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_.load())
    {
        enabled_.store(false);

        logInfo(DDSROUTER, "Stopping DDS Router.");

        // Disable thread pool so tasks running finish and new tasks are not taken by threads
        thread_pool_->disable();

        deactivate_all_topics_();
        return utils::ReturnCode::RETCODE_OK;
    }
    else
    {
        logInfo(DDSROUTER, "Trying to stop a disabled DDS Router.");
        return utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
}

void DDSRouterImpl::init_allowed_topics_()
{
    allowed_topics_ = AllowedTopicList(
        configuration_.allowlist,
        configuration_.blocklist);

    logInfo(DDSROUTER, "DDS Router configured with allowed topics: " << allowed_topics_);
}

void DDSRouterImpl::init_participants_()
{
    for (std::shared_ptr<configuration::ParticipantConfiguration> participant_config :
            configuration_.participants_configurations)
    {
        std::shared_ptr<IParticipant> new_participant;

        // Create participant
        // This should not be in try catch case as if it fails the whole init must fail
        new_participant =
                participant_factory_.create_participant(
            participant_config,
            payload_pool_,
            discovery_database_);

        // create_participant should throw an exception in fail, never return nullptr
        if (!new_participant || !new_participant->id().is_valid() ||
                new_participant->kind() == ParticipantKind::invalid)
        {
            // Failed to create participant
            throw utils::InitializationException(utils::Formatter()
                          << "Failed to create creating Participant " << participant_config->id);
        }

        logInfo(DDSROUTER, "Participant created with id: " << new_participant->id()
                                                           << " and kind " << new_participant->kind() << ".");

        // Add this participant to the database. If it is repeated it will cause an exception
        try
        {
            participants_database_->add_participant_(
                new_participant->id(),
                new_participant);
        }
        catch (const utils::InconsistencyException& )
        {
            throw utils::ConfigurationException(utils::Formatter()
                          << "Participant ids must be unique. The id " << new_participant->id() << " is duplicated.");
        }
    }

    // If DDS Router has not two or more Participants configured, it should fail
    if (participants_database_->size() < 1)
    {
        logError(DDSROUTER, "At least a Participant is required to initialize a DDS Router.");
        throw utils::InitializationException(utils::Formatter()
                      << "DDS Router requires at least 1 Participant to start.");
    }
}

void DDSRouterImpl::init_bridges_()
{
    for (std::shared_ptr<RealTopic> topic : configuration_.builtin_topics)
    {
        discovered_topic_(*topic);
    }
}

void DDSRouterImpl::discovered_topic_(
        const RealTopic& topic) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Discovered topic: " << topic << ".");

    // Check if topic already exists
    auto find_it = current_topics_.find(topic);
    if (find_it != current_topics_.end())
    {
        // If it already exists, do nothing
        return;
    }

    // Add topic to current_topics as non activated
    current_topics_.emplace(topic, false);

    // If Router is enabled and topic allowed, activate it
    if (enabled_.load() && allowed_topics_.is_topic_allowed(topic))
    {
        activate_topic_(topic);
    }
}

void DDSRouterImpl::discovered_endpoint_(
        const Endpoint& endpoint) noexcept
{
    logDebug(DDSROUTER, "Endpoint discovered in DDS Router core: " << endpoint << ".");

    discovered_topic_(endpoint.topic());
}

void DDSRouterImpl::create_new_bridge(
        const RealTopic& topic,
        bool enabled /*= false*/) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Creating Bridge for topic: " << topic << ".");

    try
    {
        bridges_[topic] = std::make_unique<Bridge>(topic, participants_database_, payload_pool_, thread_pool_, enabled);
    }
    catch (const utils::InitializationException& e)
    {
        logError(DDSROUTER,
                "Error creating Bridge for topic " << topic <<
                ". Error code:" << e.what() << ".");
    }
}

void DDSRouterImpl::activate_topic_(
        const RealTopic& topic) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Activating topic: " << topic << ".");

    // Modify current_topics_ and set this topic as active
    current_topics_[topic] = true;

    // Enable bridge. In case it is already enabled nothing should happen
    auto it_bridge = bridges_.find(topic);

    if (it_bridge == bridges_.end())
    {
        // The Bridge did not exist
        create_new_bridge(topic, true);
    }
    else
    {
        // The Bridge already exists
        it_bridge->second->enable();
    }
}

void DDSRouterImpl::deactivate_topic_(
        const RealTopic& topic) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Deactivating topic: " << topic << ".");

    // Modify current_topics_ and set this topic as non active
    current_topics_[topic] = false;

    // Disable bridge. In case it is already disabled nothing should happen
    auto it_bridge = bridges_.find(topic);

    if (it_bridge != bridges_.end())
    {
        // The Bridge already exists
        it_bridge->second->disable();
    }
    // If the Bridge does not exist, is not need to create it
}

void DDSRouterImpl::activate_all_topics_() noexcept
{
    for (auto it : current_topics_)
    {
        // Activate all topics allowed
        if (allowed_topics_.is_topic_allowed(it.first))
        {
            activate_topic_(it.first);
        }
    }
}

void DDSRouterImpl::deactivate_all_topics_() noexcept
{
    for (auto it : current_topics_)
    {
        // Deactivate all topics
        deactivate_topic_(it.first);
    }
}

} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

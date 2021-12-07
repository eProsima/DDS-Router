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
 * @file DDSRouter.cpp
 *
 */

#include <cassert>

#include <ddsrouter/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter/core/DDSRouter.hpp>
#include <ddsrouter/exceptions/UnsupportedException.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/types/Log.hpp>

namespace eprosima {
namespace ddsrouter {

// TODO: Use initial topics to start execution and start bridges

DDSRouter::DDSRouter(
        const DDSRouterConfiguration& configuration)
    : payload_pool_(new CopyPayloadPool())
    , participants_database_(new ParticipantsDatabase())
    , discovery_database_(new DiscoveryDatabase())
    , allowed_topics_()
    , bridges_()
    , configuration_(configuration)
    , participant_factory_()
    , enabled_(false)
{
    logDebug(DDSROUTER, "Creating DDS Router.");

    // Init topic allowed
    init_allowed_topics_();
    // Load Participants
    init_participants_();
    // Create Bridges
    init_bridges_();

    logDebug(DDSROUTER, "DDS Router created.");
}

DDSRouter::~DDSRouter()
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
            logWarning(DDSROUTER, "Error poping participant from database.");
        }
        else
        {
            participant_factory_.remove_participant(participant);
        }
    }

    // There is no need to destroy shared ptrs as they will delete itslefs with 0 references

    logDebug(DDSROUTER, "DDS Router destroyed.");
}

ReturnCode DDSRouter::reload_configuration(
        const DDSRouterConfiguration& new_configuration)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logDebug(DDSROUTER, "Reloading DDS Router configuration...");

    // Load new configuration and check it is okey
    AllowedTopicList new_allowed_topic_list(
        new_configuration.allowlist(),
        new_configuration.blocklist());

    // Check if it should change or is the same configuration
    if (new_allowed_topic_list == allowed_topics_)
    {
        logDebug(DDSROUTER, "Same configuration, do nothing in reload.");
        return ReturnCode::RETCODE_NO_DATA;
    }

    // Set new Allowed list
    allowed_topics_ = new_allowed_topic_list;

    logDebug(DDSROUTER, "New DDS Router allowed topics configuration: " << allowed_topics_);

    // TODO refactor with discovery functionality
    // TODO add bridge creation when initial topics configuration added

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

    return ReturnCode::RETCODE_OK;
}

ReturnCode DDSRouter::start() noexcept
{
    ReturnCode ret = start_();
    if (ret == ReturnCode::RETCODE_OK)
    {
        logUser(DDSROUTER, "Starting DDS Router.");
    }
    else if (ret == ReturnCode::RETCODE_PRECONDITION_NOT_MET)
    {
        logUser(DDSROUTER, "Trying to start a disabled DDS Router.");
    }

    return ret;
}

ReturnCode DDSRouter::stop() noexcept
{
    ReturnCode ret = stop_();
    if (ret == ReturnCode::RETCODE_OK)
    {
        logUser(DDSROUTER, "Stopping DDS Router.");
    }
    else if (ret == ReturnCode::RETCODE_PRECONDITION_NOT_MET)
    {
        logUser(DDSROUTER, "Trying to stop a not enabled DDS Router.");
    }

    return ret;
}

ReturnCode DDSRouter::start_() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!enabled_.load())
    {
        enabled_.store(true);

        logInfo(DDSROUTER, "Starting DDS Router.");

        activate_all_topics_();
        return ReturnCode::RETCODE_OK;
    }
    else
    {
        logInfo(DDSROUTER, "Trying to stop a not enabled DDS Router.");
        return ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
}

ReturnCode DDSRouter::stop_() noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (enabled_.load())
    {
        enabled_.store(false);

        logInfo(DDSROUTER, "Stopping DDS Router.");

        deactivate_all_topics_();
        return ReturnCode::RETCODE_OK;
    }
    else
    {
        logInfo(DDSROUTER, "Trying to start a disabled DDS Router.");
        return ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
}

void DDSRouter::init_allowed_topics_()
{
    allowed_topics_ = AllowedTopicList(
        configuration_.allowlist(),
        configuration_.blocklist());

    logInfo(DDSROUTER, "DDS Router configured with allowed topics: " << allowed_topics_);
}

void DDSRouter::init_participants_()
{
    for (ParticipantConfiguration participant_config :
            configuration_.participants_configurations())
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
                !new_participant->type().is_valid())
        {
            // Failed to create participant
            throw InitializationException(utils::Formatter()
                          << "Failed to create creating Participant " << participant_config.id());
        }

        logInfo(DDSROUTER, "Participant created with id: " << new_participant->id()
                                                           << " and type " << new_participant->type() << ".");

        // Add this participant to the database
        participants_database_->add_participant_(
            new_participant->id(),
            new_participant);
    }
}

void DDSRouter::init_bridges_()
{
    for (RealTopic topic : configuration_.real_topics())
    {
        discovered_topic_(topic);
    }
}

void DDSRouter::discovered_topic_(
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

void DDSRouter::create_new_bridge(
        const RealTopic& topic,
        bool enabled /*= false*/) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Creating Bridge for topic: " << topic << ".");

    try
    {
        bridges_[topic] = std::make_unique<Bridge>(topic, participants_database_, payload_pool_, enabled);
    }
    catch (const InitializationException& e)
    {
        logError(DDSROUTER,
                "Error creating Bridge for topic " << topic <<
                ". Error code:" << e.what() << ".");
    }
}

void DDSRouter::activate_topic_(
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

void DDSRouter::deactivate_topic_(
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

void DDSRouter::activate_all_topics_() noexcept
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

void DDSRouter::deactivate_all_topics_() noexcept
{
    for (auto it : current_topics_)
    {
        // Deactivate all topics
        deactivate_topic_(it.first);
    }
}

} /* namespace ddsrouter */
} /* namespace eprosima */

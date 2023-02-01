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

#include <cpp_utils/exception/UnsupportedException.hpp>
#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/Log.hpp>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>

#include <core/DDSRouterImpl.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {

using namespace eprosima::ddsrouter::core::types;

// TODO: Use initial topics to start execution and start bridges

DDSRouterImpl::DDSRouterImpl(
        const configuration::DDSRouterConfiguration& configuration,
        const std::shared_ptr<DiscoveryDatabase>& discovery_database,
        const std::shared_ptr<core::PayloadPool>& payload_pool,
        const std::shared_ptr<ParticipantsDatabase>& participants_database)
    : participants_database_(participants_database)
    , discovery_database_(discovery_database)
    , payload_pool_(payload_pool)
    , configuration_(configuration)
    , enabled_(false)
    , thread_pool_(std::make_shared<utils::SlotThreadPool>(configuration_.advanced_options.number_of_threads))
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

    // Set default value for history
    types::TopicQoS::default_history_depth.store(
        configuration_.advanced_options.max_history_depth);

    // Add callback to be called by the discovery database when an Endpoint is discovered
    discovery_database_->add_endpoint_discovered_callback(std::bind(&DDSRouterImpl::discovered_endpoint_, this,
            std::placeholders::_1));

    // Add callback to be called by the discovery database when an Endpoint is removed/dropped
    discovery_database_->add_endpoint_erased_callback(std::bind(&DDSRouterImpl::removed_endpoint_, this,
            std::placeholders::_1));

    // Init topic allowed
    init_allowed_topics_();
    // Load Participants
    init_participants_();
    // Create Bridges for builtin topics
    init_bridges_();

    // Init discovery database
    // The entities should not be added to the Discovery Database until the builtin topics have been created.
    // This is due to the fact that the Participants endpoints start discovering topics with different configuration
    // than the one specified in the yaml configuration file.
    // TODO (custompart): this does not do anything, check
    discovery_database_->start();


    logDebug(DDSROUTER, "DDS Router created.");
}

DDSRouterImpl::~DDSRouterImpl()
{
    logDebug(DDSROUTER, "Destroying DDS Router.");

    // Stop Discovery Database
    discovery_database_->stop();

    // Stop all communications
    stop_();

    // Destroy Bridges, so Writers and Readers are destroyed before the Databases
    bridges_.clear();

    // Destroy RPCBridges, so Writers and Readers are destroyed before the Databases
    rpc_bridges_.clear();

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
        std::set<DdsTopic> new_builtin_topics;
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

        // It must change the configuration. Check every topic discovered and activate/deactivate it if needed.
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
        for (DdsTopic topic : new_builtin_topics)
        {
            discovered_topic_(topic);
        }

        // Check every service discovered and activate/deactivate it if needed.
        for (auto& service_it : current_services_)
        {
            if (allowed_topics_.is_service_allowed(service_it.first))
            {
                service_it.second = true;
                rpc_bridges_[service_it.first]->enable();
            }
            else
            {
                service_it.second = false;
                rpc_bridges_[service_it.first]->disable();
            }
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

        // Enable services discovered while router disabled
        for (auto it : current_services_)
        {
            // Enable only allowed services
            if (it.second)
            {
                rpc_bridges_[it.first]->enable();
            }
        }

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
    for (auto& participant : participants_database_->get_participants_map())
    {
        participant.second->start();
    }
}

void DDSRouterImpl::init_bridges_()
{
    for (std::shared_ptr<DdsTopic> topic : configuration_.builtin_topics)
    {
        discovered_topic_(*topic);
    }
}

void DDSRouterImpl::discovered_topic_(
        const DdsTopic& topic) noexcept
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

void DDSRouterImpl::discovered_service_(
        const RPCTopic& topic,
        const ParticipantId& server_participant_id,
        const GuidPrefix& server_guid_prefix) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Discovered service: " << topic << ".");

    auto it_bridge = rpc_bridges_.find(topic);

    if (it_bridge == rpc_bridges_.end())
    {
        // Create RPCBridge even if topic not allowed, as we need to store server in database
        create_new_service(topic);

        if (allowed_topics_.is_service_allowed(topic))
        {
            current_services_[topic] = true;
        }
        else
        {
            current_services_[topic] = false;
        }
    }

    rpc_bridges_[topic]->discovered_service(server_participant_id, server_guid_prefix);
    if (enabled_.load() && current_services_[topic])
    {
        rpc_bridges_[topic]->enable();
    }
}

void DDSRouterImpl::removed_service_(
        const RPCTopic& topic,
        const ParticipantId& server_participant_id,
        const GuidPrefix& server_guid_prefix) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Removed service: " << topic << ".");

    auto it_bridge = rpc_bridges_.find(topic);

    if (it_bridge != rpc_bridges_.end())
    {
        rpc_bridges_[topic]->removed_service(server_participant_id, server_guid_prefix);
    }
}

void DDSRouterImpl::discovered_endpoint_(
        const Endpoint& endpoint) noexcept
{
    logDebug(DDSROUTER, "Endpoint discovered in DDS Router core: " << endpoint << ".");

    // Set as discovered only if the endpoint is a Reader
    // If non Readers in topics, it is considered as non discovered
    if (endpoint.is_reader())
    {
        DdsTopic topic = endpoint.topic();
        if (!RPCTopic::is_service_topic(topic))
        {
            discovered_topic_(topic);
        }
        else if (endpoint.is_server_endpoint())
        {
            // Service server discovered
            discovered_service_(RPCTopic(topic), endpoint.discoverer_participant_id(), endpoint.guid().guid_prefix());
        }
    }
}

void DDSRouterImpl::removed_endpoint_(
        const Endpoint& endpoint) noexcept
{
    logDebug(DDSROUTER, "Endpoint removed/dropped: " << endpoint << ".");

    DdsTopic topic = endpoint.topic();
    if (RPCTopic::is_service_topic(topic))
    {
        if (endpoint.is_server_endpoint())
        {
            // Service server removed/dropped
            removed_service_(RPCTopic(topic), endpoint.discoverer_participant_id(), endpoint.guid().guid_prefix());
        }
    }
}

void DDSRouterImpl::create_new_bridge(
        const DdsTopic& topic,
        bool enabled /*= false*/) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Creating Bridge for topic: " << topic << ".");

    try
    {
        bridges_[topic] = std::make_unique<DDSBridge>(topic, participants_database_, payload_pool_, thread_pool_,
                        enabled);
    }
    catch (const utils::InitializationException& e)
    {
        logError(DDSROUTER,
                "Error creating Bridge for topic " << topic <<
                ". Error code:" << e.what() << ".");
    }
}

void DDSRouterImpl::create_new_service(
        const RPCTopic& topic) noexcept
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    logInfo(DDSROUTER, "Creating Service: " << topic << ".");

    // Endpoints not created until enabled for the first time, so no exception can be thrown
    rpc_bridges_[topic] = std::make_unique<RPCBridge>(topic, participants_database_, payload_pool_, thread_pool_);
}

void DDSRouterImpl::activate_topic_(
        const DdsTopic& topic) noexcept
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
        const DdsTopic& topic) noexcept
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

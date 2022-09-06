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
 * @file CommonParticipant.cpp
 */

#include <memory>

#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/RTPSDomain.h>

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_core/types/dds/DomainId.hpp>

#include <reader/implementations/rtps/Reader.hpp>
#include <writer/implementations/rtps/Writer.hpp>
#include <participant/implementations/auxiliar/BaseParticipant.hpp>
#include <participant/implementations/rtps/CommonParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

CommonParticipant::CommonParticipant(
        std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database,
        const types::DomainId& domain_id,
        const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes,
        unsigned int max_history_depth)
    : BaseParticipant(participant_configuration, payload_pool, discovery_database)
    , max_history_depth_(max_history_depth)
    , target_guids_writer_filter_(std::make_shared<types::GuidPrefixDataFilterType>())
{
    create_participant_(
        domain_id,
        participant_attributes);

    // Add this same Participant to be filtered by the Writer
    add_filter_guidprefix_(rtps_participant_->getGuid().guidPrefix);
}

CommonParticipant::~CommonParticipant()
{
    if (rtps_participant_)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSParticipant(rtps_participant_);
    }
}

void CommonParticipant::onParticipantDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::ParticipantDiscoveryInfo&& info)
{
    if (info.info.m_guid.guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
        {
            // Check whether this Participant belongs to a local Dds Router and is such, filter its sources
            if (is_local_ddsrouter_participant_(info))
            {
                // Found local DDS Router, so filtering it
                add_filter_guidprefix_(info.info.m_guid.guidPrefix);
                logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY,
                            "Found in Participant " << this->id_nts_() <<
                            " new Local Participant " << info.info.m_guid <<
                            " that belongs to a DDS Router, so filtering it.");
            }
            else
            {
                logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY,
                        "Found in Participant " << this->id_nts_() << " new Participant " << info.info.m_guid << ".");
            }
        }
        else if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::CHANGED_QOS_PARTICIPANT)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Participant " << info.info.m_guid << " changed QoS.");
        }
        else if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Participant " << info.info.m_guid << " removed.");
        }
        else
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Participant " << info.info.m_guid << " dropped.");
        }
    }
}

template<class DiscoveryInfoKind>
types::Endpoint CommonParticipant::create_endpoint_from_info_(
        DiscoveryInfoKind& info)
{
    // Parse GUID
    types::Guid info_guid;
    info_guid = info.info.guid();

    // Parse QoS
    types::DurabilityKind info_durability_kind = info.info.m_qos.m_durability.durabilityKind();
    types::ReliabilityKind info_reliability_kind;
    if (info.info.m_qos.m_reliability.kind == fastdds::dds::BEST_EFFORT_RELIABILITY_QOS)
    {
        info_reliability_kind = fastrtps::rtps::BEST_EFFORT;
    }
    else if (info.info.m_qos.m_reliability.kind == fastdds::dds::RELIABLE_RELIABILITY_QOS)
    {
        info_reliability_kind = fastrtps::rtps::RELIABLE;
    }
    else
    {
        utils::tsnh(
            utils::Formatter() <<
                "Invalid ReliabilityQoS value found while parsing DiscoveryInfo for Endpoint creation.");
    }
    types::QoS info_qos(info_durability_kind, info_reliability_kind);

    // Parse Topic
    types::RealTopic info_topic(std::string(info.info.topicName()), std::string(info.info.typeName()),
            info.info.topicKind() == eprosima::fastrtps::rtps::TopicKind_t::WITH_KEY);

    // Create Endpoint
    if (std::is_same<DiscoveryInfoKind, fastrtps::rtps::ReaderDiscoveryInfo>::value)
    {
        return types::Endpoint(types::EndpointKind::reader, info_guid, info_qos, info_topic);
    }
    else if (std::is_same<DiscoveryInfoKind, fastrtps::rtps::WriterDiscoveryInfo>::value)
    {
        return types::Endpoint(types::EndpointKind::writer, info_guid, info_qos, info_topic);
    }
    else
    {
        utils::tsnh(utils::Formatter() << "Invalid DiscoveryInfoKind for Endpoint creation.");
        return types::Endpoint();
    }
}

void CommonParticipant::onReaderDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::ReaderDiscoveryInfo&& info)
{
    if (info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        types::Endpoint info_reader = create_endpoint_from_info_<fastrtps::rtps::ReaderDiscoveryInfo>(info);

        if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Reader " << info.info.guid() << ".");

            this->discovery_database_->add_endpoint(info_reader);
        }
        else if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Reader " << info.info.guid() << " changed QoS.");

            this->discovery_database_->update_endpoint(info_reader);
        }
        else if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Reader " << info.info.guid() << " removed.");

            info_reader.active(false);
            this->discovery_database_->update_endpoint(info_reader);
        }
        else
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Reader " << info.info.guid() << " dropped.");

            info_reader.active(false);
            this->discovery_database_->update_endpoint(info_reader);
        }
    }
}

void CommonParticipant::onWriterDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::WriterDiscoveryInfo&& info)
{
    if (info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        types::Endpoint info_writer = create_endpoint_from_info_<fastrtps::rtps::WriterDiscoveryInfo>(info);

        if (info.status == fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Writer " << info.info.guid() << ".");

            this->discovery_database_->add_endpoint(info_writer);
        }
        else if (info.status == fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Writer " << info.info.guid() << " changed QoS.");

            this->discovery_database_->update_endpoint(info_writer);
        }
        else if (info.status == fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Writer " << info.info.guid() << " removed.");

            info_writer.active(false);
            this->discovery_database_->update_endpoint(info_writer);
        }
        else
        {
            logInfo(DDSROUTER_RTPSPARTICIPANT_DISCOVERY, "Writer " << info.info.guid() << " dropped.");

            info_writer.active(false);
            this->discovery_database_->update_endpoint(info_writer);
        }
    }
}

void CommonParticipant::create_participant_(
        const types::DomainId& domain,
        const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes)
{
    logInfo(DDSROUTER_RTPS_PARTICIPANT,
            "Creating Participant in domain " << domain);

    rtps_participant_ = fastrtps::rtps::RTPSDomain::createParticipant(
        domain,
        participant_attributes);

    // Set listener after participant creation to avoid SEGFAULT (produced when callback using rtps_participant_ is
    // invoked before the variable is fully set)
    rtps_participant_->set_listener(this);

    if (!rtps_participant_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating RTPS Participant " << this->id());
    }

    logInfo(DDSROUTER_RTPS_PARTICIPANT,
            "New Participant " << this->configuration_->kind <<
            " created with id " << this->id() <<
            " in domain " << domain << " with guid " << rtps_participant_->getGuid() <<
            (this->is_repeater() ? " (repeater)" : " (non repeater)"));
}

std::shared_ptr<IWriter> CommonParticipant::create_writer_(
        types::RealTopic topic)
{
    return std::make_shared<Writer>(
        this->id(),
        topic,
        this->payload_pool_,
        rtps_participant_,
        max_history_depth_,
        target_guids_writer_filter_,
        this->configuration_->is_repeater);
}

std::shared_ptr<IReader> CommonParticipant::create_reader_(
        types::RealTopic topic)
{
    return std::make_shared<Reader>(
        this->id(),
        topic,
        this->payload_pool_,
        rtps_participant_);
}

void CommonParticipant::add_filter_guidprefix_(const types::GuidPrefix& guid_to_filter) noexcept
{
    // Lock to write
    std::unique_lock<types::GuidPrefixDataFilterType> lock(*target_guids_writer_filter_);
    target_guids_writer_filter_->insert(guid_to_filter);
}

fastrtps::rtps::RTPSParticipantAttributes
CommonParticipant::participant_attributes_(
        const configuration::ParticipantConfiguration* participant_configuration)
{
    fastrtps::rtps::RTPSParticipantAttributes params;

    // Add Participant name
    params.setName(participant_configuration->id.id_name().c_str());

    // Set property so other Routers know the Participants belongs to a Router and its kind
    eprosima::fastrtps::rtps::Property router_kind_property;
    router_kind_property.name(std::string(ROUTER_PROPERTY_KIND_NAME_));
    router_kind_property.value(std::string(types::PARTICIPANT_KIND_STRINGS[static_cast<int>(participant_configuration->kind)]));
    router_kind_property.propagate(true);
    // Add it to properties
    params.properties.properties().push_back(router_kind_property);

    // Set property so other Routers know the Participant is local or wan
    eprosima::fastrtps::rtps::Property router_positioning_property;
    router_positioning_property.name(std::string(ROUTER_PROPERTY_POSITIONING_NAME_));
    router_positioning_property.value(
        std::string(
            (participant_configuration->kind == types::ParticipantKind::local_discovery_server ||
                    participant_configuration->kind == types::ParticipantKind::simple_rtps)
                ? ROUTER_PROPERTY_POSITIONING_VALUE_LOCAL_
                : ROUTER_PROPERTY_POSITIONING_VALUE_WAN_));
    router_positioning_property.propagate(true);
    // Add it to properties
    params.properties.properties().push_back(router_positioning_property);

    return params;
}

bool CommonParticipant::is_local_ddsrouter_participant_(const fastrtps::rtps::ParticipantDiscoveryInfo& info) noexcept
{

    // Check if router positioning property exist
    for (auto pit = info.info.m_properties.begin();
         pit != info.info.m_properties.end();
         ++pit)
    {
        // Attribute found
        if (pit->first() == std::string(ROUTER_PROPERTY_POSITIONING_NAME_))
        {
            return pit->second() == std::string(ROUTER_PROPERTY_POSITIONING_VALUE_LOCAL_);
        }
    }

    return false;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

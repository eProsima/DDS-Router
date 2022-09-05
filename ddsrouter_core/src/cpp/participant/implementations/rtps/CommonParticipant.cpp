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
        const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes)
    : BaseParticipant(participant_configuration, payload_pool, discovery_database)
{
    create_participant_(
        domain_id,
        participant_attributes);
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
            logInfo(DDSROUTER_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Participant " << info.info.m_guid << ".");
        }
        else if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::CHANGED_QOS_PARTICIPANT)
        {
            logInfo(DDSROUTER_DISCOVERY, "Participant " << info.info.m_guid << " changed QoS.");
        }
        else if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT)
        {
            logInfo(DDSROUTER_DISCOVERY, "Participant " << info.info.m_guid << " removed.");
        }
        else
        {
            logInfo(DDSROUTER_DISCOVERY, "Participant " << info.info.m_guid << " dropped.");
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
    types::QoS info_qos;
    // Durability
    info_qos.durability_qos = info.info.m_qos.m_durability.durabilityKind();
    // Reliability
    if (info.info.m_qos.m_reliability.kind == fastdds::dds::BEST_EFFORT_RELIABILITY_QOS)
    {
        info_qos.reliability_qos = fastrtps::rtps::BEST_EFFORT;
    }
    else if (info.info.m_qos.m_reliability.kind == fastdds::dds::RELIABLE_RELIABILITY_QOS)
    {
        info_qos.reliability_qos = fastrtps::rtps::RELIABLE;
    }
    else
    {
        utils::tsnh(
            utils::Formatter() <<
                "Invalid ReliabilityQoS value found while parsing DiscoveryInfo for Endpoint creation.");
    }

    // History
    info_qos.history_qos.depth = info.info.m_qos.m_durabilityService.history_depth;
    info_qos.history_qos.kind = info.info.m_qos.m_durabilityService.history_kind;

    // TODO: Set partitions
    // NOTE: dont know where partitions are going to be used

    // Parse Topic
    types::DdsTopic info_topic(std::string(info.info.topicName()), std::string(info.info.typeName()));
    info_topic.keyed = info.info.topicKind() == eprosima::fastrtps::rtps::TopicKind_t::WITH_KEY;
    info_topic.topic_qos.set_value(info_qos, utils::FuzzyLevel::fuzzy_set);

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
    info.info.m_qos.m_durabilityService.history_kind;
    if (info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        types::Endpoint info_reader = create_endpoint_from_info_<fastrtps::rtps::ReaderDiscoveryInfo>(info);

        if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        {
            logInfo(DDSROUTER_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Reader " << info.info.guid() << ".");

            this->discovery_database_->add_endpoint(info_reader);
        }
        else if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER)
        {
            logInfo(DDSROUTER_DISCOVERY, "Reader " << info.info.guid() << " changed QoS.");

            this->discovery_database_->update_endpoint(info_reader);
        }
        else if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        {
            logInfo(DDSROUTER_DISCOVERY, "Reader " << info.info.guid() << " removed.");

            info_reader.active(false);
            this->discovery_database_->update_endpoint(info_reader);
        }
        else
        {
            logInfo(DDSROUTER_DISCOVERY, "Reader " << info.info.guid() << " dropped.");

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
            logInfo(DDSROUTER_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Writer " << info.info.guid() << ".");

            this->discovery_database_->add_endpoint(info_writer);
        }
        else if (info.status == fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER)
        {
            logInfo(DDSROUTER_DISCOVERY, "Writer " << info.info.guid() << " changed QoS.");

            this->discovery_database_->update_endpoint(info_writer);
        }
        else if (info.status == fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        {
            logInfo(DDSROUTER_DISCOVERY, "Writer " << info.info.guid() << " removed.");

            info_writer.active(false);
            this->discovery_database_->update_endpoint(info_writer);
        }
        else
        {
            logInfo(DDSROUTER_DISCOVERY, "Writer " << info.info.guid() << " dropped.");

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
        types::DdsTopic topic)
{
    return std::make_shared<Writer>(
        this->id(),
        topic,
        this->payload_pool_,
        rtps_participant_,
        this->configuration_->is_repeater);
}

std::shared_ptr<IReader> CommonParticipant::create_reader_(
        types::DdsTopic topic)
{
    return std::make_shared<Reader>(
        this->id(),
        topic,
        this->payload_pool_,
        rtps_participant_);
}

fastrtps::rtps::RTPSParticipantAttributes
CommonParticipant::participant_attributes_(
        const configuration::ParticipantConfiguration* participant_configuration)
{
    fastrtps::rtps::RTPSParticipantAttributes params;

    params.setName(participant_configuration->id.id_name().c_str());

    return params;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

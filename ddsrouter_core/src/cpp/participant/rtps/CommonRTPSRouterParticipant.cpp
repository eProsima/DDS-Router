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
 * @file CommonRTPSRouterParticipant.cpp
 */

#include <memory>

#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/RTPSDomain.h>

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_core/types/dds/DomainId.hpp>

#include <reader/rtps/Reader.hpp>
#include <writer/rtps/Writer.hpp>
#include <participant/rtps/CommonRTPSRouterParticipant.hpp>
#include <configuration/participant/SimpleParticipantConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

void CommonRTPSRouterParticipant::RTPSParticipantDeleter::operator ()(
        fastrtps::rtps::RTPSParticipant* ptr) const
{
    if (ptr)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSParticipant(ptr);
    }
}

CommonRTPSRouterParticipant::CommonRTPSRouterParticipant(
        const types::ParticipantId& id)
    : IParticipant(id)
{
    throw utils::InitializationException(
              utils::Formatter() << "RTPS participant needs a configuration and a discovery_database");
}

CommonRTPSRouterParticipant::CommonRTPSRouterParticipant(
        const configuration::ParticipantConfiguration& participant_configuration,
        DiscoveryDatabase& discovery_database)
    : IParticipant(participant_configuration, discovery_database)
{
    if (dynamic_cast<const configuration::SimpleParticipantConfiguration*>(this->configuration_) == nullptr)
    {
        throw utils::InitializationException(
                  utils::Formatter() <<
                      "ParticipantConfiguration not castable to SimpleParticipantConfiguration");
    }
}

CommonRTPSRouterParticipant::~CommonRTPSRouterParticipant()
{
}

void CommonRTPSRouterParticipant::onParticipantDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::ParticipantDiscoveryInfo&& info)
{
    if (info.info.m_guid.guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT,
                    "Found in Participant " << this->id() << " new Participant " << info.info.m_guid << ".");
        }
        else if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::CHANGED_QOS_PARTICIPANT)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Participant " << info.info.m_guid << " changed QoS.");
        }
        else if (info.status == fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Participant " << info.info.m_guid << " removed.");
        }
        else
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Participant " << info.info.m_guid << " dropped.");
        }
    }
}

template <typename DiscoveryInfoKind>
types::Endpoint CommonRTPSRouterParticipant::create_endpoint_from_info_(
        DiscoveryInfoKind& discovery_info)
{
    // Parse GUID
    types::Guid info_guid;
    info_guid = discovery_info.info.guid();

    // Parse QoS
    types::DurabilityKind info_durability_kind = discovery_info.info.m_qos.m_durability.durabilityKind();
    types::ReliabilityKind info_reliability_kind;
    if (discovery_info.info.m_qos.m_reliability.kind == fastdds::dds::BEST_EFFORT_RELIABILITY_QOS)
    {
        info_reliability_kind = fastrtps::rtps::BEST_EFFORT;
    }
    else if (discovery_info.info.m_qos.m_reliability.kind == fastdds::dds::RELIABLE_RELIABILITY_QOS)
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
    types::RealTopic info_topic(std::string(discovery_info.info.topicName()), std::string(
                discovery_info.info.typeName()),
            discovery_info.info.topicKind() == eprosima::fastrtps::rtps::TopicKind_t::WITH_KEY, false);

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
        throw utils::InitializationException(utils::Formatter() << "Invalid DiscoveryInfoKind for Endpoint creation.");
    }
}

void CommonRTPSRouterParticipant::onReaderDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::ReaderDiscoveryInfo&& discovery_info)
{
    if (discovery_info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        types::Endpoint endpoint_reader =
                create_endpoint_from_info_<fastrtps::rtps::ReaderDiscoveryInfo>(discovery_info);

        if (discovery_info.status == fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT,
                    "Found in Participant " << this->id() << " new Reader " << discovery_info.info.guid() << ".");

            this->discovery_database_->add_endpoint(endpoint_reader);
        }
        else if (discovery_info.status == fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Reader " << discovery_info.info.guid() << " changed QoS.");

            this->discovery_database_->update_endpoint(endpoint_reader);
        }
        else if (discovery_info.status == fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Reader " << discovery_info.info.guid() << " removed.");

            endpoint_reader.active(false);
            this->discovery_database_->update_endpoint(endpoint_reader);
        }
        else
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Reader " << discovery_info.info.guid() << " dropped.");

            endpoint_reader.active(false);
            this->discovery_database_->update_endpoint(endpoint_reader);
        }
    }
}

void CommonRTPSRouterParticipant::onWriterDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::WriterDiscoveryInfo&& discovery_info)
{
    if (discovery_info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        types::Endpoint endpoint_writer =
                create_endpoint_from_info_<fastrtps::rtps::WriterDiscoveryInfo>(discovery_info);

        if (discovery_info.status == fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT,
                    "Found in Participant " << this->id() << " new Writer " << discovery_info.info.guid() << ".");

            this->discovery_database_->add_endpoint(endpoint_writer);
        }
        else if (discovery_info.status == fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Writer " << discovery_info.info.guid() << " changed QoS.");

            this->discovery_database_->update_endpoint(endpoint_writer);
        }
        else if (discovery_info.status == fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Writer " << discovery_info.info.guid() << " removed.");

            endpoint_writer.active(false);
            this->discovery_database_->update_endpoint(endpoint_writer);
        }
        else
        {
            logInfo(DDSROUTER_RTPS_PARTICIPANT, "Writer " << discovery_info.info.guid() << " dropped.");

            endpoint_writer.active(false);
            this->discovery_database_->update_endpoint(endpoint_writer);
        }
    }
}

void CommonRTPSRouterParticipant::create_participant_()
{
    if (rtps_participant_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating RTPS Participant. Already created " << this->id());
    }

    types::DomainId domain =
            static_cast<const configuration::SimpleParticipantConfiguration*>(this->configuration_)->domain();
    fastrtps::rtps::RTPSParticipantAttributes params = participant_attributes_();

    logInfo(DDSROUTER_RTPS_PARTICIPANT,
            "Creating Participant in domain " << domain);


    rtps_participant_ = std::unique_ptr<fastrtps::rtps::RTPSParticipant, RTPSParticipantDeleter>(
        fastrtps::rtps::RTPSDomain::createParticipant(domain, params),
        RTPSParticipantDeleter()
        );

    if (!rtps_participant_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating RTPS Participant " << this->id());
    }

    // Set listener after participant creation to avoid SEGFAULT (produced when callback using rtps_participant_ is
    // invoked before the variable is fully set)
    rtps_participant_->set_listener(this);

    logInfo(DDSROUTER_RTPS_PARTICIPANT,
            "New Participant " << this->id() <<
            " in domain " << domain << " with guid " << rtps_participant_->getGuid());
}

std::unique_ptr<IWriter> CommonRTPSRouterParticipant::create_writer_(
        const types::RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool)
{
    return std::make_unique<Writer>(
        this->id(), topic,
        payload_pool, rtps_participant_.get());
}

std::unique_ptr<IReader> CommonRTPSRouterParticipant::create_reader_(
        const types::RealTopic& topic,
        std::shared_ptr<fastrtps::rtps::IPayloadPool>& payload_pool,
        DataForwardQueue& data_forward_queue)
{
    return std::make_unique<Reader>(
        this->id(), topic,
        payload_pool, data_forward_queue, rtps_participant_.get());
}

fastrtps::rtps::RTPSParticipantAttributes
CommonRTPSRouterParticipant::participant_attributes_() const
{
    fastrtps::rtps::RTPSParticipantAttributes params;
    return params;
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */


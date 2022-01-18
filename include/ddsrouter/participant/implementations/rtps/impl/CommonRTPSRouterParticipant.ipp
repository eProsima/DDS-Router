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
 * @file EchoParticipant.cpp
 */

#ifndef _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_COMMONRTPSROUTERPARTICIPANT_HPP_
#define _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_COMMONRTPSROUTERPARTICIPANT_HPP_

#include <memory>

#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include <fastrtps/rtps/RTPSDomain.h>

#include <ddsrouter/reader/implementations/rtps/Reader.hpp>
#include <ddsrouter/writer/implementations/rtps/Writer.hpp>
#include <ddsrouter/exceptions/InitializationException.hpp>
#include <ddsrouter/participant/implementations/auxiliar/BaseParticipant.hpp>

namespace eprosima {
namespace ddsrouter {
namespace rtps {

template <class ConfigurationType>
CommonRTPSRouterParticipant<ConfigurationType>::CommonRTPSRouterParticipant(
        std::shared_ptr<ConfigurationType> participant_configuration,
        std::shared_ptr<PayloadPool> payload_pool,
        std::shared_ptr<DiscoveryDatabase> discovery_database)
    : BaseParticipant<ConfigurationType>(participant_configuration, payload_pool, discovery_database)
{
    // init_();
}

template <class ConfigurationType>
CommonRTPSRouterParticipant<ConfigurationType>::~CommonRTPSRouterParticipant()
{
    if (rtps_participant_)
    {
        fastrtps::rtps::RTPSDomain::removeRTPSParticipant(rtps_participant_);
    }
}

template <class ConfigurationType>
void CommonRTPSRouterParticipant<ConfigurationType>::onParticipantDiscovery(
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
        else
        {
            logInfo(DDSROUTER_DISCOVERY, "Participant " << info.info.m_guid << " dropped.");
        }
    }
}

template <class ConfigurationType>
void CommonRTPSRouterParticipant<ConfigurationType>::onReaderDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::ReaderDiscoveryInfo&& info)
{
    if (info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        {
            logInfo(DDSROUTER_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Reader " << info.info.guid() << ".");
        }
        else if (info.status == fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER)
        {
            logInfo(DDSROUTER_DISCOVERY, "Reader " << info.info.guid() << " changed QoS.");
        }
        else
        {
            logInfo(DDSROUTER_DISCOVERY, "Reader " << info.info.guid() << " dropped.");
        }
    }
}

template <class ConfigurationType>
void CommonRTPSRouterParticipant<ConfigurationType>::onWriterDiscovery(
        fastrtps::rtps::RTPSParticipant*,
        fastrtps::rtps::WriterDiscoveryInfo&& info)
{
    if (info.info.guid().guidPrefix != this->rtps_participant_->getGuid().guidPrefix)
    {
        if (info.status == fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        {
            logInfo(DDSROUTER_DISCOVERY,
                    "Found in Participant " << this->id_nts_() << " new Writer " << info.info.guid() << ".");
        }
        else if (info.status == fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER)
        {
            logInfo(DDSROUTER_DISCOVERY, "Writer " << info.info.guid() << " changed QoS.");
        }
        else
        {
            logInfo(DDSROUTER_DISCOVERY, "Writer " << info.info.guid() << " dropped.");
        }
    }
}

template <class ConfigurationType>
void CommonRTPSRouterParticipant<ConfigurationType>::create_participant_()
{
    DomainId domain = this->configuration_->domain();
    fastrtps::rtps::RTPSParticipantAttributes params = participant_attributes_();

    logInfo(DDSROUTER_RTPS_PARTICIPANT,
            "Creating Participant in domain " << domain);

    rtps_participant_ = fastrtps::rtps::RTPSDomain::createParticipant(domain(), params, this);
    if (!rtps_participant_)
    {
        throw InitializationException(
                  utils::Formatter() << "Error creating RTPS Participant " << this->id());
    }

    logInfo(DDSROUTER_RTPS_PARTICIPANT,
            "New Participant " << this->configuration_->type() <<
            " created with id " << this->id() <<
            " in domain " << domain << " with guid " << rtps_participant_->getGuid());
}

template <class ConfigurationType>
std::shared_ptr<IWriter> CommonRTPSRouterParticipant<ConfigurationType>::create_writer_(
        RealTopic topic)
{
    return std::make_shared<Writer>(
        this->id(), topic,
        this->payload_pool_, rtps_participant_);
}

template <class ConfigurationType>
std::shared_ptr<IReader> CommonRTPSRouterParticipant<ConfigurationType>::create_reader_(
        RealTopic topic)
{
    return std::make_shared<Reader>(this->id(), topic, this->payload_pool_, rtps_participant_);
}

template <class ConfigurationType>
fastrtps::rtps::RTPSParticipantAttributes
CommonRTPSRouterParticipant<ConfigurationType>::participant_attributes_() const
{
    fastrtps::rtps::RTPSParticipantAttributes params;
    return params;
}

} /* namespace rtps */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_PARTICIPANT_IMPLEMENTATIONS_AUX_COMMONRTPSROUTERPARTICIPANT_HPP_ */

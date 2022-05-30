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
 * @file CustomLocatorReader.cpp
 */

#include <random>

#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipant.h>

#include <reader/implementations/rtps/CustomLocatorReader.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

using namespace eprosima::ddsrouter::core::types;

CustomLocatorReader::CustomLocatorReader(
        const ParticipantId& participant_id,
        const RealTopic& topic,
        std::shared_ptr<PayloadPool> payload_pool,
        fastrtps::rtps::RTPSParticipant* rtps_participant)
    : Reader(participant_id, topic, payload_pool, rtps_participant)
{
    // Create History
    fastrtps::rtps::HistoryAttributes history_att = history_attributes_();
    rtps_history_ = new fastrtps::rtps::ReaderHistory(history_att);

    // Create Reader
    fastrtps::rtps::ReaderAttributes reader_att = reader_attributes_();

    // Create different locator for each Reader regarding the default unicast locator of Participant
    // int i = 0;
    // for (auto locator : rtps_participant->getRTPSParticipantAttributes().defaultUnicastLocatorList)
    // {
    //     locator.port = 0,
    //     reader_att.endpoint.unicastLocatorList.push_back(locator);
    //     logDebug(
    //         DDSROUTER_RTPS_READER,
    //         "Adding locator to Reader " << topic << " from Participant: " << participant_id << " : " << locator);
    // }
    fastrtps::rtps::Locator_t locator;
    locator.port = 7000 + (std::rand() % 10000);
    reader_att.endpoint.unicastLocatorList.push_back(locator);

    rtps_reader_ = fastrtps::rtps::RTPSDomain::createRTPSReader(
        rtps_participant,
        reader_att,
        payload_pool_,
        rtps_history_);

    for (auto locator : rtps_reader_->getAttributes().unicastLocatorList)
    {
        logError(DEBUG, "Locators in Reader: " << locator);
    }

    // Set listener after entity creation to avoid SEGFAULT (produced when callback using rtps_reader_ is
    // invoked before the variable is fully set)
    rtps_reader_->setListener(this);

    if (!rtps_reader_)
    {
        throw utils::InitializationException(
                  utils::Formatter() << "Error creating Simple RTPSReader for Participant " <<
                      participant_id << " in topic " << topic << ".");
    }

    // Register reader with topic
    fastrtps::TopicAttributes topic_att = topic_attributes_();
    fastrtps::ReaderQos reader_qos = reader_qos_();

    if (!rtps_participant->registerReader(rtps_reader_, topic_att, reader_qos))
    {
        // In case it fails, remove reader and throw exception
        fastrtps::rtps::RTPSDomain::removeRTPSReader(rtps_reader_);
        throw utils::InitializationException(utils::Formatter() << "Error registering topic " << topic <<
                      " for Simple RTPSReader in Participant " << participant_id);
    }

    logInfo(DDSROUTER_RTPS_READER, "New Reader created in Participant " << participant_id_ << " for topic " <<
            topic << " with guid " << rtps_reader_->getGuid());
}

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

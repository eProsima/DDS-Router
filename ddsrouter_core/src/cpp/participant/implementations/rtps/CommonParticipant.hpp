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
 * @file CommonParticipant.hpp
 */

#ifndef __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONPARTICIPANT_HPP_
#define __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONPARTICIPANT_HPP_

#include <fastdds/rtps/participant/ParticipantDiscoveryInfo.h>
#include <fastdds/rtps/reader/ReaderDiscoveryInfo.h>
#include <fastdds/rtps/rtps_fwd.h>
#include <fastdds/rtps/writer/WriterDiscoveryInfo.h>
#include <fastrtps/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/rtps/participant/RTPSParticipantListener.h>

#include <ddsrouter_core/configuration/participant/ParticipantConfiguration.hpp>
#include <ddsrouter_core/types/dds/DomainId.hpp>

#include <participant/implementations/auxiliar/BaseParticipant.hpp>
#include <reader/implementations/rtps/Reader.hpp>
#include <writer/implementations/rtps/Writer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * TODO
 */
class CommonParticipant
    : public BaseParticipant
    , public fastrtps::rtps::RTPSParticipantListener
{
public:

    CommonParticipant(
            std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database,
            const types::DomainId& domain_id,
            const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes,
            unsigned int max_history_depth);

    virtual ~CommonParticipant();

    virtual void onParticipantDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::ParticipantDiscoveryInfo&& info) override;

    virtual void onReaderDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::ReaderDiscoveryInfo&& info) override;

    virtual void onWriterDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::WriterDiscoveryInfo&& info) override;

protected:

    void create_participant_(
            const types::DomainId& domain,
            const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes);

    std::shared_ptr<IWriter> create_writer_(
            types::RealTopic topic) override;

    std::shared_ptr<IReader> create_reader_(
            types::RealTopic topic) override;

    template<class DiscoveryInfoKind>
    types::Endpoint create_endpoint_from_info_(
            DiscoveryInfoKind& info);

    void add_filter_guidprefix_(const types::GuidPrefix& guid_to_filter) noexcept;

    /////
    // RTPS specific methods

    static fastrtps::rtps::RTPSParticipantAttributes participant_attributes_(
            const configuration::ParticipantConfiguration* participant_configuration);

    static bool is_local_ddsrouter_participant_(const fastrtps::rtps::ParticipantDiscoveryInfo& info) noexcept;

    /////
    // VARIABLES
    eprosima::fastrtps::rtps::RTPSParticipant* rtps_participant_;

    //! Mutex that guards every access to the RTPS Participant
    mutable std::recursive_mutex rtps_mutex_;

    //! Maximum depth of RTPS History instances
    unsigned int max_history_depth_;

    /**
     * @brief Collects these guid prefixes that the Writer must filter and not send messages to them.
     *
     * It does add also this same Participant.
     */
    std::shared_ptr<types::GuidPrefixDataFilterType> target_guids_writer_filter_;

    /*
     * These constants set the name of property and values used in Property QoS
     */
    static constexpr const char* ROUTER_PROPERTY_KIND_NAME_ = "fastdds.ddsrouter.kind";
    static constexpr const char* ROUTER_PROPERTY_POSITIONING_NAME_ = "fastdds.ddsrouter.positioning";
    static constexpr const char* ROUTER_PROPERTY_POSITIONING_VALUE_LOCAL_ = "local";
    static constexpr const char* ROUTER_PROPERTY_POSITIONING_VALUE_WAN_ = "wan";
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONPARTICIPANT_HPP_ */


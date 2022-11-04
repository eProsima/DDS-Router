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

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace rtps {

/**
 * Abstract generic class for a RTPS Participant wrapper.
 *
 * Concrete classes that inherit from this would only need to specialize specific methods related with the
 * qos and attributes.
 *
 * @warning This object is not RAII and must be initialized before used.
 */
class CommonParticipant
    : public BaseParticipant
    , public fastrtps::rtps::RTPSParticipantListener
{
public:

    /**
     * @brief Construct a CommonParticipant
     */
    CommonParticipant(
            std::shared_ptr<configuration::ParticipantConfiguration> participant_configuration,
            std::shared_ptr<PayloadPool> payload_pool,
            std::shared_ptr<DiscoveryDatabase> discovery_database,
            const types::DomainId& domain_id,
            const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes);

    //! Remove internal RTPS Participant
    virtual ~CommonParticipant();

    /**
     * @brief Create the internal RTPS Participant using the attributes given.
     *
     * @attention this method should be called right after constructor to create enable internal entities.
     * This is required as this object is a Listener that could be called before finishing construction.
     * Other alternatives have been studied but none have really fit for this case.
     *
     * @throw InitializationException if RTPS Participant creation fails
     *
     * @warning this method is not thread safe.
     * @pre this method can only be called once.
     */
    void init();

    /**
     * @brief Override method from \c RTPSParticipantListener .
     *
     * This method only is for debugging purposes.
     */
    virtual void onParticipantDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::ParticipantDiscoveryInfo&& info) override;

    /**
     * @brief Override method from \c RTPSParticipantListener .
     *
     * This method adds to database the endpoint discovered or modified.
     */
    virtual void onReaderDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::ReaderDiscoveryInfo&& info) override;

    /**
     * @brief Override method from \c RTPSParticipantListener .
     *
     * This method adds to database the endpoint discovered or modified.
     */
    virtual void onWriterDiscovery(
            fastrtps::rtps::RTPSParticipant* participant,
            fastrtps::rtps::WriterDiscoveryInfo&& info) override;

protected:

    /**
     * @brief Auxiliary method to create the internal RTPS participant.
     */
    void create_participant_(
            const types::DomainId& domain,
            const fastrtps::rtps::RTPSParticipantAttributes& participant_attributes);

    /**
     * @brief Create a writer object
     *
     * Depending on the Topic QoS creates a Basic or Specific Writer.
     */
    std::shared_ptr<IWriter> create_writer_(
            types::DdsTopic topic) override;

    /**
     * @brief Create a reader object
     *
     * Depending on the Topic QoS creates a Basic or Specific Reader.
     */
    std::shared_ptr<IReader> create_reader_(
            types::DdsTopic topic) override;

    /**
     * @brief Create a endpoint from info object
     *
     * Specialized for \c WriterDiscoveryInfo and \c ReaderDiscoveryInfo .
     */
    template<class DiscoveryInfoKind>
    types::Endpoint create_endpoint_from_info_(
            DiscoveryInfoKind& info);

    //! Create a endpoint from common info from method \c create_endpoint_from_info_ .
    template<class DiscoveryInfoKind>
    types::Endpoint create_common_endpoint_from_info_(
            DiscoveryInfoKind& info);

    /////
    // RTPS specific methods

    /**
     * @brief Static method that gives the std attributes for a Participant.
     *
     * @note This method must be specialized from inherit classes.
     */
    static fastrtps::rtps::RTPSParticipantAttributes get_participant_attributes_(
            const configuration::ParticipantConfiguration* participant_configuration);

    /////
    // VARIABLES
    //! Internal RTPS Participant
    eprosima::fastrtps::rtps::RTPSParticipant* rtps_participant_;

    types::DomainId domain_id_;

    fastrtps::rtps::RTPSParticipantAttributes participant_attributes_;
};

} /* namespace rtps */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* __SRC_DDSROUTERCORE_PARTICIPANT_IMPLEMENTATIONS_RTPS_COMMONPARTICIPANT_HPP_ */

// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file MockParticipant.hpp
 */

#pragma once

#include <ddsrouter_core/participants/participant/configuration/ParticipantConfiguration.hpp>
#include <ddsrouter_core/participants/participant/auxiliar/BaseParticipant.hpp>
#include <ddsrouter_core/participants/reader/auxiliar/MockReader.hpp>

namespace eprosima {
namespace ddsrouter {
namespace participants {

/**
 * TODO
 */
class MockParticipant : public BaseParticipant
{
public:

    /**
     * TODO
     */
    MockParticipant(
            std::shared_ptr<ParticipantConfiguration> participant_configuration,
            std::shared_ptr<core::DiscoveryDatabase> discovery_database);

    /**
     * @brief Destroy the Mock Participant object
     */
    virtual ~MockParticipant();

    /**
     * @brief Simulate that this Participant has discovered a new endpoint
     *
     * @param new_endpoint : Endpoint discovered
     */
    void simulate_discovered_endpoint(
            const core::types::Endpoint& new_endpoint);

    std::shared_ptr<MockReader> get_reader(
            core::types::DdsTopic topic);

    std::vector<MockDataStored> get_writer(
            core::types::DdsTopic topic);

protected:

    //! Override create_writer_() BaseParticipant method
    std::shared_ptr<core::IWriter> create_writer_(
            core::types::DdsTopic topic) override;

    //! Override create_reader_() BaseParticipant method
    std::shared_ptr<core::IReader> create_reader_(
            core::types::DdsTopic topic) override;
};

} /* namespace participants */
} /* namespace ddsrouter */
} /* namespace eprosima */

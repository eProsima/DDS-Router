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

#pragma once

#include <ddspipe_core/interface/IReader.hpp>
#include <ddspipe_core/interface/IWriter.hpp>
#include <ddspipe_core/interface/IParticipant.hpp>
#include <ddspipe_core/interface/ITopic.hpp>
#include <ddspipe_core/interface/IRoutingData.hpp>
#include <ddspipe_core/testing/entities/blank_entities.hpp>

namespace eprosima {
namespace ddspipe {
namespace core {
namespace testing {

struct MockParticipant : public BlankParticipant
{
    types::ParticipantId id() const noexcept override;

    std::shared_ptr<IWriter> create_writer(
            const ITopic& topic) override;

    std::shared_ptr<IReader> create_reader(
            const ITopic& topic) override;
};

struct MockReader : public BlankReader
{
    void enable() noexcept override;

    void disable() noexcept override;

    void set_on_data_available_callback(
            std::function<void()> on_data_available_lambda) noexcept override;

    void unset_on_data_available_callback() noexcept override;

    utils::ReturnCode take(
            std::unique_ptr<IRoutingData>& data) noexcept override;

    core::types::Guid guid() const override;

    fastrtps::RecursiveTimedMutex& get_rtps_mutex() const override;

    uint64_t get_unread_count() const override;

    types::DdsTopic topic() const override;

    types::ParticipantId participant_id() const override;
};

struct MockReader : public BlankWriter
{
    void enable() noexcept override;

    void disable() noexcept override;

    utils::ReturnCode write(
            IRoutingData& data) noexcept override;
};

struct MockTopic : public BlankTopic
{
    types::TopicInternalTypeDiscriminator internal_type_discriminator() const noexcept override;
};

struct MockRoutingData : public BlankRoutingData
{
    types::TopicInternalTypeDiscriminator internal_type_discriminator() const noexcept override;
};

const    types::TopicInternalTypeDiscriminator INTERNAL_TOPIC_TYPE_TEST = "testing_type::mock::v0";

} /* namespace testing */
} /* namespace core */
} /* namespace ddspipe */
} /* namespace eprosima */

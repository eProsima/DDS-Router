// Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <atomic>
#include <thread>

#include <fastdds/dds/core/detail/DDSReturnCode.hpp>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/testing/gtest_aux.hpp>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/XmlParticipantConfiguration.hpp>

#include <ddsrouter_core/core/DdsRouter.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

#include <test_participants.hpp>

#include <gtest/gtest.h>

using namespace eprosima;
using namespace eprosima::ddspipe;
using namespace eprosima::ddsrouter::core;

namespace test {

constexpr const uint32_t DEFAULT_SAMPLES_TO_RECEIVE = 5;
constexpr const uint32_t DEFAULT_MILLISECONDS_PUBLISH_LOOP = 100;
constexpr const uint32_t DEFAULT_MESSAGE_SIZE = 1; // x50 bytes

// Maximum time to wait for the forwarded dispose to be received by the subscriber.
constexpr const uint32_t MAX_MILLISECONDS_WAIT_DISPOSE = 5000;
constexpr const uint32_t MILLISECONDS_WAIT_STEP = 100;

/**
 * @brief Create a DDS Router configuration that bridges \c TOPIC_NAME between:
 *   - a Simple participant in domain 0 (publisher side), and
 *   - an XML (DDS) participant in domain 1 (subscriber side) configured as a REPEATER.
 *
 * The XML participant is the one that exercises the buggy code path: as a repeater its
 * DataWriter installs a \c dds::RepeaterDataFilter prefilter. When the dispose/unregister
 * sample produced by the publisher is forwarded to the subscriber through this writer, the
 * prefilter is evaluated for a sample whose \c user_write_data is null (dispose/unregister
 * are sent through DataWriter::dispose / unregister_instance, which cannot carry WriteParams).
 *
 * Note: no participant_profile is set on the XML participant, so XmlParticipant falls back to
 * the configured domain (1) — this keeps the test self-contained (no external XML profile file).
 */
DdsRouterConfiguration xml_repeater_dispose_configuration()
{
    DdsRouterConfiguration conf;

    // -- Filter the test topic by name ---------------------------------------
    // (any type, keyed or not)
    core::types::WildcardDdsFilterTopic topic;
    topic.topic_name.set_value(TOPIC_NAME);
    conf.ddspipe_configuration.allowlist.insert(
        utils::Heritable<core::types::WildcardDdsFilterTopic>::make_heritable(topic));

    // -- Create the participants ---------------------------------------------

    // Simple participant in domain 0 (publisher side)
    {
        auto part = std::make_shared<participants::SimpleParticipantConfiguration>();
        part->id = core::types::ParticipantId("simple_participant_0");
        part->domain.domain_id = 0u;
        conf.participants_configurations.insert({types::ParticipantKind::simple, part});
    }

    // XML (DDS) participant in domain 1 (subscriber side), in REPEATER mode.
    {
        auto part = std::make_shared<participants::XmlParticipantConfiguration>();
        part->id = core::types::ParticipantId("xml_repeater_participant_1");
        part->domain.domain_id = 1u;
        part->is_repeater = true;
        conf.participants_configurations.insert({types::ParticipantKind::xml, part});
    }

    return conf;
}

/**
 * Regression test for the segfault in dds::RepeaterDataFilter::evaluate when forwarding a
 * dispose/unregister sample (null user_write_data) through a repeater XML participant.
 *
 * Without the fix the router process crashes (SIGSEGV) while forwarding the dispose, so this
 * test aborts. With the fix the dispose is forwarded and received by the subscriber.
 */
void test_xml_repeater_key_dispose(
        DdsRouterConfiguration ddsrouter_configuration,
        uint32_t samples_to_receive = DEFAULT_SAMPLES_TO_RECEIVE,
        uint32_t time_between_samples = DEFAULT_MILLISECONDS_PUBLISH_LOOP,
        uint32_t msg_size = DEFAULT_MESSAGE_SIZE)
{
    uint32_t samples_sent = 0;
    std::atomic<uint32_t> samples_received(0);

    HelloWorldKeyed msg;
    HelloWorldKeyedPubSubType type;

    std::string msg_str;
    for (uint32_t i = 0; i < msg_size; i++)
    {
        msg_str += "Testing DdsRouter Blackbox Repeater Dispose ...";
    }
    msg.message(msg_str);
    msg.id(666);

    // -- Create keyed Publisher, Subscriber & Router -------------------------

    // Publisher (domain 0)
    TestPublisher<HelloWorldKeyed> publisher(type.is_compute_key_provided);
    ASSERT_TRUE(publisher.init(0));

    // Subscriber (domain 1)
    TestSubscriber<HelloWorldKeyed> subscriber(type.is_compute_key_provided);
    ASSERT_TRUE(subscriber.init(1, &msg, &samples_received));

    // Create and start the DDS Router
    DdsRouter router(ddsrouter_configuration);
    router.start();

    // -- Publish some messages -----------------------------------------------
    while (samples_received.load() < samples_to_receive)
    {
        msg.index(++samples_sent);
        ASSERT_EQ(publisher.publish(msg), fastdds::dds::RETCODE_OK) << samples_sent;

        if (time_between_samples > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_samples));
        }
    }

    // -- Dispose the keyed publisher -----------------------------------------

    // The dispose is routed to the XML repeater participant's writer,
    // whose RepeaterDataFilter prefilter is evaluated with a null user_write_data.
    // >>> Without the fix the router segfaults at this point. <<<
    ASSERT_EQ(publisher.dispose_key(msg), fastdds::dds::RETCODE_OK);

    // With the fix, the dispose is considered relevant and forwarded to the subscriber
    uint32_t waited = 0;
    while (subscriber.n_disposed() < 1u && waited < MAX_MILLISECONDS_WAIT_DISPOSE)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(MILLISECONDS_WAIT_STEP));
        waited += MILLISECONDS_WAIT_STEP;
    }

    ASSERT_GE(subscriber.n_disposed(), 1u);

    router.stop();
}

} /* namespace test */

/**
 * Test that a dispose forwarded through a repeater XML (DDS) participant does not crash the router
 * and is correctly received by the subscriber.
 *
 * Regression test for the null user_write_data dereference in dds::RepeaterDataFilter::evaluate.
 */
TEST(DDSTestRepeaterDisposeKey, end_to_end_xml_repeater_key_dispose)
{
    test::test_xml_repeater_key_dispose(
        test::xml_repeater_dispose_configuration());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

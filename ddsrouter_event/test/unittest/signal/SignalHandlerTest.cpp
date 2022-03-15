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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <ddsrouter_utils/utils.hpp>

#include <ddsrouter_event/SignalHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {
namespace test {

constexpr uint32_t N_DEFAUL_TEST_EXECUTIONS = 5;

} /* namespace test */
} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

using namespace eprosima::ddsrouter;
using namespace eprosima::ddsrouter::event;

/**
 * Create SignaHandlers individually and let them destroy
 *
 * CASES:
 * - SIGINT with callback
 * - SIGINT without callback
 * - SIGTERM with callback
 * - SIGTERM without callback
 */
TEST(SignalHandlerTest, create_signal_handler)
{
    // SIGINT with callback
    {
        SignalHandler<SIGNAL_SIGINT> handler;
    }

    // SIGINT without callback
    {
        SignalHandler<SIGNAL_SIGINT> handler;
    }

    // SIGTERM with callback
    {
        SignalHandler<SIGNAL_SIGTERM> handler;
    }

    // SIGTERM without callback
    {
        SignalHandler<SIGNAL_SIGTERM> handler;
    }
}

/**
 * Test creating cohexistent signal handlers for same and different signals
 *
 * CASES:
 * - N signal handlers of SIGINT
 * - N signal handlers of SIGTERM
 * - N signal handlers of SIGINT & SIGTERM
 */
TEST(SignalHandlerTest, create_several_signal_handlers)
{
    // N signal handlers of SIGINT
    {
        std::vector<std::unique_ptr<SignalHandler<SIGNAL_SIGINT>>> v;
        for (int i=0; i<test::N_DEFAUL_TEST_EXECUTIONS; i++)
        {
            v.push_back(std::make_unique<SignalHandler<SIGNAL_SIGINT>>());
        }
    }

    // N signal handlers of SIGTERM
    {
        std::vector<std::unique_ptr<SignalHandler<SIGNAL_SIGTERM>>> v;
        for (int i=0; i<test::N_DEFAUL_TEST_EXECUTIONS; i++)
        {
            v.push_back(std::make_unique<SignalHandler<SIGNAL_SIGTERM>>());
        }
    }

    // N signal handlers of SIGINT & SIGTERM
    {
        std::vector<std::unique_ptr<IBaseSignalHandler>> v;
        for (int i=0; i<test::N_DEFAUL_TEST_EXECUTIONS; i++)
        {
            v.push_back(std::make_unique<SignalHandler<SIGNAL_SIGINT>>());
            v.push_back(std::make_unique<SignalHandler<SIGNAL_SIGTERM>>());
        }
    }
}

/**
 * Receive a signal from a Signal handler
 *
 * CASES:
 * - SIGUSR1
 * - SIGUSR1 double handler
 * - SIGUSR1 double signal
 * - SIGUSR1 receive signal after unset
 * - SIGUSR1 receive signal just before destroying
 */
TEST(SignalHandlerTest, receive_signal)
{
    // SIGUSR1
    {
        // Number of calls to the signal
        uint32_t calls = 0;

        // Create signal handler
        SignalHandler<SIGNAL_SIGUSR1> handler( [&calls](int /* signal_number */ ){ calls++; } );

        // Raise signal
        std::raise(SIGUSR1);

        // Force handler to wait for signal
        handler.wait_for_event();

        // Check that signal has been received
        ASSERT_EQ(1, calls);
    }

    // SIGUSR1 double handler
    {
        // Number of calls to the signal
        uint32_t calls = 0;

        // Create signal handler
        SignalHandler<SIGNAL_SIGUSR1> handler1 ( [&calls](int /* signal_number */ ){ calls++; } );
        SignalHandler<SIGNAL_SIGUSR1> handler2 ( [&calls](int /* signal_number */ ){ calls++; } );

        // Raise signal
        std::raise(SIGUSR1);

        // Force handler to wait for signal
        handler1.wait_for_event();
        handler2.wait_for_event();

        // Check that signal has been received
        ASSERT_EQ(2, calls);
    }

    // SIGUSR1 double signal
    {
        // Number of calls to the signal
        uint32_t calls = 0;

        // Create signal handler
        SignalHandler<SIGNAL_SIGUSR1> handler( [&calls](int /* signal_number */ ){ calls++; } );

        // Raise signal
        std::raise(SIGUSR1);
        std::raise(SIGUSR1);

        // Force handler to wait for signal
        handler.wait_for_event(2);

        // Check that signal has been received
        ASSERT_EQ(2, calls);
    }

    // SIGUSR1 receive signal after unset
    {
        // Number of calls to the signal
        uint32_t calls = 0;

        // Create signal handler
        SignalHandler<SIGNAL_SIGUSR1> handler( [&calls](int /* signal_number */ ){ calls++; } );

        // Raise signal
        std::raise(SIGUSR1);

        // Force handler to wait for signal
        handler.wait_for_event();

        // Check that signal has been received
        ASSERT_EQ(1, calls);

        // Unset callback, so new signal raise is not listened

        handler.unset_callback();

        // Raise signal
        std::raise(SIGUSR1);

        // Check that signal has not been received
        ASSERT_EQ(1, calls);
    }

    // SIGUSR1 receive signal just before destroying
    {
        {
            // Create signal handler
            SignalHandler<SIGNAL_SIGUSR1> handler( [](int /* signal_number */ ){ /* empty callback */ } );
        }

        // Destroying handler while Raise signal
        std::raise(SIGUSR1);
    }
}

/**
 * Receive N signals and handled them
 *
 * CASES:
 * - 2
 * - 20
 * - 200
 */
TEST(SignalHandlerTest, receive_n_signals)
{
    std::vector<uint32_t> cases({2u, 20u, 200u});

    // Cases
    for (uint32_t number_signals : cases)
    {
        // Number of calls to the signal
        uint32_t calls = 0;

        // Create signal handler
        SignalHandler<SIGNAL_SIGUSR1> handler( [&calls](int /* signal_number */ ){ calls++; } );

        // Raise N signal
        for (uint32_t i=0; i<number_signals; i++)
        {
            std::raise(SIGUSR1);
        }

        // Force handler to wait for signal
        handler.wait_for_event(number_signals);

        // Check that signal has been received
        ASSERT_EQ(number_signals, calls);
    }
}

/**
 * Create 2 handlers for same signal, get singal from both, then unset a callback and get signal in just one
 *
 * This tests that the set and unset of callbacks is done correctly
 */
TEST(SignalHandlerTest, erase_callback_while_other_handling)
{
    // Number of calls to the signal
    uint32_t calls = 0;

    // Create signal handler
    // This adds 10 each time signal is called
    SignalHandler<SIGNAL_SIGUSR1> handler1 ( [&calls](int /* signal_number */ ){ calls += 10; } );
    // This adds 1 each time signal is called
    SignalHandler<SIGNAL_SIGUSR1> handler2 ( [&calls](int /* signal_number */ ){ calls += 1; } );

    // Raise signal
    std::raise(SIGUSR1);

    // Force handler to wait for signal
    handler1.wait_for_event();
    handler2.wait_for_event();

    // Check that signal has been received by both
    ASSERT_EQ(11, calls);

    // Unset handler 1 (adds 10)
    handler1.unset_callback();

    // Raise signal
    std::raise(SIGUSR1);

    // Force handler2 (remaining) to wait for signal (2 because it is the second one)
    handler2.wait_for_event(2);

    // Check that signal has been received by only 2 (that adds 1)
    ASSERT_EQ(12, calls);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

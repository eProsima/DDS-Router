// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file TestLogHandler.cpp
 *
 */

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <TestLogHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace test {

TestLogHandler::TestLogHandler(
        utils::Log::Kind threshold, /* Log::Kind::Warning */
        uint32_t expected_severe_logs /* = 0 */,
        uint32_t max_severe_logs /* = 0 */)
    : log_consumer_(
        new event::LogSevereEventHandler(
            [](utils::Log::Entry entry)
            {
            },
            threshold))
    , expected_severe_logs_(expected_severe_logs)
    , max_severe_logs_(std::max(max_severe_logs, expected_severe_logs)) // Use max to avoid forcing set both args
{
}

void TestLogHandler::check_valid()
{
    ASSERT_GE(log_consumer_->event_count(), expected_severe_logs_);
    ASSERT_LE(log_consumer_->event_count(), max_severe_logs_);
}

TestLogHandler::~TestLogHandler()
{
    utils::Log::Flush();
    check_valid();
    utils::Log::Reset();
}

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

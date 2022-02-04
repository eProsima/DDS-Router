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
        Log::Kind threshold, /* Log::Kind::Warning */
        uint32_t max_severe_logs /* = 0 */)
    : log_consumer_(new event::LogSevereEventHandler([](eprosima::fastdds::dds::Log::Entry entry)
            {
            }, threshold))
    , max_severe_logs_(max_severe_logs)
{
}

void TestLogHandler::check_valid()
{
    ASSERT_LE(log_consumer_->event_count(), max_severe_logs_);
}

TestLogHandler::~TestLogHandler()
{
    check_valid();
    Log::Reset();
}

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

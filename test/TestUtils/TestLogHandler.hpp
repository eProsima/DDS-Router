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
 * @file TestLogHandler.hpp
 */

#ifndef _DDSROUTER_TEST_TESTUTILS_TESTLOGHANDLER_HPP_
#define _DDSROUTER_TEST_TESTUTILS_TESTLOGHANDLER_HPP_

#include <ddsrouter/event/LogEventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace test {

class TestLogHandler
{
public:
    TestLogHandler(
        uint32_t max_severe_logs = 0);

    ~TestLogHandler();

    void check_valid();
protected:
    uint32_t max_severe_logs_;
    event::LogSevereEventHandler* log_consumer_;
};

} /* namespace test */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TEST_TESTUTILS_TESTLOGHANDLER_HPP_ */

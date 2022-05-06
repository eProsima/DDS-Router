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
 * @file ExecutionTimerDebugger.cpp
 */

#include <ddsrouter_utils/performance_debugger/ExecutionTimerDebugger.hpp>
#include <ddsrouter_utils/Log.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace debug {

#if EPROSIMA_PERFORMANCE_DEBUG

ExecutionTimerDebugger::ExecutionTimerDebugger(const std::string& debugger_name)
    : timer_(false)
    , debugger_name_(debugger_name)
{
}

ExecutionTimerDebugger::~ExecutionTimerDebugger()
{
    double total_time = timer_.elapsed();
    double execution_time = timer_.elapsed_active();
    double proportion = timer_.active_proportion();

    logPerformance(
        PERFORMANCE,
        "ExecutionTimerDebugger <" << debugger_name_ << "> has lived for " << total_time << " seconds. " <<
        "It has worked for " << execution_time << " seconds. Thus it has worked  " <<
        (100*proportion) << "\% of it total life time.");
}

void ExecutionTimerDebugger::in_execution()
{
    timer_.activate();
}

void ExecutionTimerDebugger::end_execution()
{
    timer_.deactivate();
}

#endif // if EPROSIMA_PERFORMANCE_DEBUG

} /* namespace debug */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

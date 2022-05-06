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
 * @file ExecutionTimerDebugger.hpp
 */

#ifndef _DDSROUTERUTILS_EFFICIENCYDEBUGGER_EXECUTIONTIMERDEBUGGER_HPP_
#define _DDSROUTERUTILS_EFFICIENCYDEBUGGER_EXECUTIONTIMERDEBUGGER_HPP_

#include <string>

#include <ddsrouter_utils/time/ProportionalTimer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {
namespace debug {

/*
 * The idea behind the class implemented in this file is to measure how much time is a class or function
 * is executed in comparison with the total time of the class existing.
 *
 * It is thought to be the most efficiency possible when \c EPROSIMA_PERFORMANCE_DEBUG is not defined, by
 * implemented an empty class (compiler optimizes it) and the methods do anything.
 *
 * NOTE: There was another way to do this, by forcing the user to create and use this class from macros,
 * in a way that each macro is empty when \c EPROSIMA_PERFORMANCE_DEBUG is not defined, and call what it must
 * call otherwise.
 * This was proben to be a bit more efficient than the current way (just a bit) but much more difficult to
 * use and to implement.
 */

#ifdef EPROSIMA_PERFORMANCE_DEBUG

/**
 * TODO
 */
class ExecutionTimerDebugger
{
public:

    ExecutionTimerDebugger(const std::string& debugger_name);

    ~ExecutionTimerDebugger();

    void in_execution();

    void end_execution();

protected:

    ProportionalTimer timer_;
    std::string debugger_name_;
};

#else

/**
 * Empty class that does not do anything and has no variables (so compiler does not create it)
 * Reimplement the methods from \c ExecutionTimerDebugger so the do not do anything and thus
 * the call is very fast even when no using class.
 */
class ExecutionTimerDebugger
{
public:
    ExecutionTimerDebugger(const std::string& ) {}
    inline void in_execution() {}
    inline void end_execution() {}
};

#endif // if EPROSIMA_PERFORMANCE_DEBUG

} /* namespace debug */
} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_EFFICIENCYDEBUGGER_EXECUTIONTIMERDEBUGGER_HPP_ */

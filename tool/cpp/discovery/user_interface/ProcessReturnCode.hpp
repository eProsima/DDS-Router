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
 * @file ProcessReturnCode.hpp
 *
 */

#ifndef _DDSROUTER_TOOL_CPP_DISCOVERY_USERINTERFACE_PROCESSRETURNCODE_HPP_
#define _DDSROUTER_TOOL_CPP_DISCOVERY_USERINTERFACE_PROCESSRETURNCODE_HPP_

namespace eprosima {
namespace ddsrouter {
namespace discovery {
namespace ui {

enum ProcessReturnCode
{
    SUCCESS = 0,
    HELP_ARGUMENT = 1,
    INCORRECT_ARGUMENT = 10,
    REQUIRED_ARGUMENT_FAILED = 11,
    EXECUTION_FAILED = 20,
    CONFIGURATION_FAILED = 21,
    INITIALIZATION_FAILED = 22,
};

} /* namespace ui */
} /* namespace discovery */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_TOOL_CPP_DISCOVERY_USERINTERFACE_PROCESSRETURNCODE_HPP_ */

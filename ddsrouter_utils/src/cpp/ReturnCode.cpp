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
 * @file ReturnCode.cpp
 *
 */

#include <assert.h>
#include <ddsrouter_utils/ReturnCode.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

const std::map<ReturnCode, std::string> ReturnCode::to_string_conversion_ =
{
    {ReturnCode::RETCODE_OK, "Ok"},
    {ReturnCode::RETCODE_ERROR, "Error"},
    {ReturnCode::RETCODE_UNSUPPORTED, "Unsupported"},
    {ReturnCode::RETCODE_BAD_PARAMETER, "BadParameter"},
    {ReturnCode::RETCODE_PRECONDITION_NOT_MET, "PreconditionNotMet"},
    {ReturnCode::RETCODE_OUT_OF_RESOURCES, "OutOfResources"},
    {ReturnCode::RETCODE_NOT_ENABLED, "NotEnabled"},
    {ReturnCode::RETCODE_IMMUTABLE_POLICY, "ImmutablePolicy"},
    {ReturnCode::RETCODE_INCONSISTENT_POLICY, "IncosistentPolicy"},
    {ReturnCode::RETCODE_ALREADY_DELETED, "AlreadyDeleted"},
    {ReturnCode::RETCODE_TIMEOUT, "Timeout"},
    {ReturnCode::RETCODE_NO_DATA, "NoData"},
    {ReturnCode::RETCODE_ILLEGAL_OPERATION, "IllegalOperation"},
    {ReturnCode::RETCODE_NOT_ALLOWED_BY_SECURITY, "NotAllowedBySecurity"},
};

bool ReturnCode::operator ()() const noexcept
{
    return RETCODE_OK == *this;
}

bool ReturnCode::operator <(
        const ReturnCode& other) const noexcept
{
    return (*this)() < other();
}

std::ostream& operator <<(
        std::ostream& os,
        const ReturnCode& code)
{
    auto it = ReturnCode::to_string_conversion_.find(code);
    assert(it != ReturnCode::to_string_conversion_.end());
    os << "{" << it->second << "}";
    return os;
}

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

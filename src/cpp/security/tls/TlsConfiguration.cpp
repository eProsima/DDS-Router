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
 * @file TlsConfiguration.cpp
 *
 */

#include <ddsrouter/security/tls/TlsConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace security {

TlsConfiguration::TlsConfiguration()
    : kind_(TLS_INVALID)
{
}

TlsConfiguration::TlsConfiguration(
        TlsConfigurationKind kind)
    : kind_(kind)
{
}

bool TlsConfiguration::is_valid() const noexcept
{
    return true;
}

bool TlsConfiguration::is_active() const noexcept
{
    return false;
}

bool TlsConfiguration::can_be_client() const noexcept
{
    return false;
}

bool TlsConfiguration::can_be_server() const noexcept
{
    return false;
}

TlsConfigurationKind TlsConfiguration::tls_kind() const noexcept
{
    return kind_;
}

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

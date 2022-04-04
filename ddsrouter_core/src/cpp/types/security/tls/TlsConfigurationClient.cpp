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
 * @file TlsConfigurationClient.cpp
 *
 */

#include <ddsrouter_core/types/security/tls/TlsConfigurationClient.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {
namespace security {

TlsConfigurationClient::TlsConfigurationClient(
        const std::string& certificate_authority_file, const std::string& sni_host)
    : TlsConfiguration(TLS_CLIENT)
    , certificate_authority_file_(certificate_authority_file)
    , sni_host_(sni_host)
{
}

bool TlsConfigurationClient::is_valid() const noexcept
{
    // TODO check it is a correct file
    return certificate_authority_file_ != "" || sni_host_ != "";
}

bool TlsConfigurationClient::is_active() const noexcept
{
    return true;
}

bool TlsConfigurationClient::can_be_client() const noexcept
{
    return true;
}

bool TlsConfigurationClient::can_be_server() const noexcept
{
    return false;
}

std::string TlsConfigurationClient::certificate_authority_file() const noexcept
{
    return certificate_authority_file_;
}

std::string TlsConfigurationClient::sni_host() const noexcept
{
        return sni_host_;
}

} /* namespace security */
} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

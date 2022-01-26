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
 * @file TlsConfigurationBoth.cpp
 *
 */

#include <ddsrouter/security/tls/TlsConfigurationBoth.hpp>

namespace eprosima {
namespace ddsrouter {
namespace security {

TlsConfigurationBoth::TlsConfigurationBoth(
        const std::string& certificate_authority_file,
        const std::string& private_key_file_password,
        const std::string& private_key_file,
        const std::string& certificate_chain_file,
        const std::string& dh_params_file)
    : TlsConfiguration(TLS_BOTH)
    , TlsConfigurationClient(certificate_authority_file)
    , TlsConfigurationServer(
        private_key_file_password,
        private_key_file,
        certificate_chain_file,
        dh_params_file)
{
}

bool TlsConfigurationBoth::is_valid() const noexcept
{
    return TlsConfigurationClient::is_valid() && TlsConfigurationServer::is_valid();
}

bool TlsConfigurationBoth::is_active() const noexcept
{
    return true;
}

bool TlsConfigurationBoth::can_be_client() const noexcept
{
    return true;
}

bool TlsConfigurationBoth::can_be_server() const noexcept
{
    return true;
}

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

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

TlsConfiguration::TlsConfiguration(
        TlsConfigurationKind kind,
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_authority_file,
        std::string certificate_chain_file,
        std::string dh_params_file)
    : kind(kind)
    , private_key_file_password(private_key_file_password)
    , private_key_file(private_key_file)
    , certificate_authority_file(certificate_authority_file)
    , certificate_chain_file(certificate_chain_file)
    , dh_params_file(dh_params_file)
{
}

TlsConfiguration::TlsConfiguration()
    : kind(TLS_INVALID)
{
}

TlsConfiguration::TlsConfiguration(
        std::string certificate_authority_file)
    : kind(TLS_SERVER)
    , private_key_file_password(private_key_file_password)
    , private_key_file(private_key_file)
    , certificate_chain_file(certificate_chain_file)
    , dh_params_file(dh_params_file)
{
}

TlsConfiguration::TlsConfiguration(
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_chain_file,
        std::string dh_params_file)
    : kind(TLS_SERVER)
    , private_key_file_password(private_key_file_password)
    , private_key_file(private_key_file)
    , certificate_chain_file(certificate_chain_file)
    , dh_params_file(dh_params_file)
{
}

TlsConfiguration::TlsConfiguration(
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_authority_file,
        std::string certificate_chain_file,
        std::string dh_params_file)
    : TlsConfiguration(
        TLS_BOTH,
        private_key_file_password,
        private_key_file,
        certificate_authority_file,
        certificate_chain_file,
        dh_params_file)
{
}

bool TlsConfiguration::is_valid() const noexcept
{
    // TODO
    return true;
}

bool TlsConfiguration::is_active() const noexcept
{
    return kind != TLS_INVALID;
}

bool TlsConfiguration::tls_kind() const noexcept
{
    return kind;
}

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

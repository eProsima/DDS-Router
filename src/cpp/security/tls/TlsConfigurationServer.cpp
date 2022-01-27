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
 * @file TlsConfigurationServer.cpp
 *
 */

#include <ddsrouter/security/tls/TlsConfigurationServer.hpp>

namespace eprosima {
namespace ddsrouter {
namespace security {

TlsConfigurationServer::TlsConfigurationServer(
        const std::string& private_key_file_password,
        const std::string& private_key_file,
        const std::string& certificate_chain_file,
        const std::string& dh_params_file)
    : TlsConfiguration(TLS_SERVER)
    , private_key_file_password_(private_key_file_password)
    , private_key_file_(private_key_file)
    , certificate_chain_file_(certificate_chain_file)
    , dh_params_file_(dh_params_file)
{
}

bool TlsConfigurationServer::is_valid() const noexcept
{
    // TODO check it is a correct file
    return private_key_file_ != "" &&
           certificate_chain_file_ != "" &&
           dh_params_file_ != "";
}

bool TlsConfigurationServer::is_active() const noexcept
{
    return true;
}

bool TlsConfigurationServer::can_be_client() const noexcept
{
    return false;
}

bool TlsConfigurationServer::can_be_server() const noexcept
{
    return true;
}

std::string TlsConfigurationServer::private_key_file_password() const noexcept
{
    return private_key_file_password_;
}

std::string TlsConfigurationServer::private_key_file() const noexcept
{
    return private_key_file_;
}

std::string TlsConfigurationServer::certificate_chain_file() const noexcept
{
    return certificate_chain_file_;
}

std::string TlsConfigurationServer::dh_params_file() const noexcept
{
    return dh_params_file_;
}

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

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
 * @file TlsConfiguration.hpp
 */

#ifndef _DDSROUTER_SECURITY_TLS_TLSCONFIGURATION_HPP_
#define _DDSROUTER_SECURITY_TLS_TLSCONFIGURATION_HPP_

#include <string>

namespace eprosima {
namespace ddsrouter {
namespace security {

enum TlsConfigurationKind
{
    TLS_INVALID,
    TLS_CLIENT,
    TLS_SERVER,
    TLS_BOTH,
};

/**
 * This class joins data to configure TLS
 *
 * TODO: do it right with proper class and methods.
 */
class TlsConfiguration
{
public:

    TlsConfiguration(
        TlsConfigurationKind kind,
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_authority_file,
        std::string certificate_chain_file,
        std::string dh_params_file);

    TlsConfiguration();

    TlsConfiguration(
        std::string certificate_authority_file);

    TlsConfiguration(
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_chain_file,
        std::string dh_params_file);

    TlsConfiguration(
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_authority_file,
        std::string certificate_chain_file,
        std::string dh_params_file);

    bool is_valid() const noexcept;

    bool is_active() const noexcept;

    bool tls_kind() const noexcept;

    TlsConfigurationKind kind;
    std::string private_key_file_password;
    std::string private_key_file;
    std::string certificate_authority_file;
    std::string certificate_chain_file;
    std::string dh_params_file;
};

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_SECURITY_TLS_TLSCONFIGURATION_HPP_ */

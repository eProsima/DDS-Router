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
 * @file TlsConfigurationBoth.hpp
 */

#ifndef _DDSROUTER_SECURITY_TLS_TLSCONFIGURATIONBOTH_HPP_
#define _DDSROUTER_SECURITY_TLS_TLSCONFIGURATIONBOTH_HPP_

#include <string>

#include <ddsrouter/security/tls/TlsConfigurationServer.hpp>
#include <ddsrouter/security/tls/TlsConfigurationClient.hpp>

namespace eprosima {
namespace ddsrouter {
namespace security {

/**
 * TODO
 */
class TlsConfigurationBoth : public TlsConfigurationServer, public TlsConfigurationClient
{
public:

    TlsConfigurationBoth(
            const std::string& certificate_authority_file,
            const std::string& private_key_file_password,
            const std::string& private_key_file,
            const std::string& certificate_chain_file,
            const std::string& dh_params_file);

    virtual bool is_valid() const noexcept override;

    virtual bool is_active() const noexcept override;

    virtual bool can_be_client() const noexcept override;

    virtual bool can_be_server() const noexcept override;
};

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_SECURITY_TLS_TLSCONFIGURATIONBOTH_HPP_ */

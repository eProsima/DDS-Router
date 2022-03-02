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
 * @file TlsConfigurationServer.hpp
 */

#ifndef _DDSROUTERCORE_SECURITY_TLS_TLSCONFIGURATIONSERVER_HPP_
#define _DDSROUTERCORE_SECURITY_TLS_TLSCONFIGURATIONSERVER_HPP_

#include <string>

#include <ddsrouter_core/security/tls/TlsConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {
namespace security {

/**
 * TODO
 */
class TlsConfigurationServer : virtual public TlsConfiguration
{
public:

    TlsConfigurationServer(
            const std::string& private_key_file_password,
            const std::string& private_key_file,
            const std::string& certificate_chain_file,
            const std::string& dh_params_file);

    virtual bool is_valid() const noexcept override;

    virtual bool is_active() const noexcept override;

    virtual bool can_be_client() const noexcept override;

    virtual bool can_be_server() const noexcept override;

    std::string private_key_file_password() const noexcept;

    std::string private_key_file() const noexcept;

    std::string certificate_chain_file() const noexcept;

    std::string dh_params_file() const noexcept;

protected:

    std::string private_key_file_password_;
    std::string private_key_file_;
    std::string certificate_chain_file_;
    std::string dh_params_file_;
};

} /* namespace security */
} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERCORE_SECURITY_TLS_TLSCONFIGURATIONSERVER_HPP_ */

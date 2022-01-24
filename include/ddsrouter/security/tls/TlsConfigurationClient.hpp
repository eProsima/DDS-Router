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
 * @file TlsConfigurationClient.hpp
 */

#ifndef _DDSROUTER_SECURITY_TLS_TLSCONFIGURATIONCLIENT_HPP_
#define _DDSROUTER_SECURITY_TLS_TLSCONFIGURATIONCLIENT_HPP_

#include <string>

#include <ddsrouter/security/tls/TlsConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace security {

/**
 * TODO
 */
class TlsConfigurationClient : virtual public TlsConfiguration
{
public:

    TlsConfigurationClient(
        const std::string& certificate_authority_file);

    virtual bool is_valid() const noexcept override;

    virtual bool is_active() const noexcept override;

    virtual bool can_be_client() const noexcept override;

    virtual bool can_be_server() const noexcept override;

    std::string certificate_authority_file() const noexcept;

protected:

    std::string certificate_authority_file_;
};

} /* namespace security */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTER_SECURITY_TLS_TLSCONFIGURATIONCLIENT_HPP_ */

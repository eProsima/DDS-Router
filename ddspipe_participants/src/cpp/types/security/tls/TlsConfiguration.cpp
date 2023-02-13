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

#include <cpp_utils/exception/ConfigurationException.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/Log.hpp>
#include <cpp_utils/utils.hpp>

#include <ddspipe_participants/types/security/tls/TlsConfiguration.hpp>

namespace eprosima {
namespace ddspipe {
namespace participants {
namespace types {

bool TlsConfiguration::is_active() const noexcept
{
    return this->kind != TlsKind::inactive;
}

void TlsConfiguration::enable_tls(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
        bool client /* = false */) const
{
    // Apply security ON
    descriptor->apply_security = true;

    // Options
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::DEFAULT_WORKAROUNDS);
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::SINGLE_DH_USE);
    descriptor->tls_config.add_option(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions::NO_SSLV2); // not safe

    if (verify_peer)
    {
        // Perform verification of the server
        descriptor->tls_config.add_verify_mode(
            eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER);
    }
    else
    {
        descriptor->tls_config.add_verify_mode(
            eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_NONE);
    }

    if (client)
    {
        if (!compatible<types::TlsKind::client>())
        {
            logError(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                    "TLS Configuration expected a Client configuration.");
            throw utils::ConfigurationException("TLS Configuration expected a Client configuration.");
        }
        else
        {
            enable_tls_client(descriptor,  true);
        }
    }
    else
    {
        if (!compatible<types::TlsKind::server>())
        {
            logError(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
                    "TLS Configuration expected a Server configuration.");
            throw utils::ConfigurationException("TLS Configuration expected a Server configuration.");
        }
        else
        {
            // Add configuration for server
            enable_tls_server(descriptor);

            // In case it could also be client, add tls config
            if (compatible<types::TlsKind::client>())
            {
                enable_tls_client(descriptor,  false);
            }
        }
    }

    logDebug(DDSROUTER_DISCOVERYSERVER_PARTICIPANT,
            "TLS configured.");
}

void TlsConfiguration::enable_tls_client(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor,
        bool only_client) const
{
    if (only_client && verify_peer)
    {
        // Fail verification if the server has no certificate
        descriptor->tls_config.add_verify_mode(
            eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT);
    }

    // CA certificate
    descriptor->tls_config.verify_file = certificate_authority_file;

    // SNI server name
    descriptor->tls_config.server_name = sni_server_name;
}

void TlsConfiguration::enable_tls_server(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor) const
{
    // Password
    descriptor->tls_config.password = private_key_file_password;
    // Private key
    descriptor->tls_config.private_key_file = private_key_file;
    // DDS-Router certificate
    descriptor->tls_config.cert_chain_file = certificate_chain_file;
    // DH
    descriptor->tls_config.tmp_dh_file = dh_params_file;
}

template <>
DDSPIPE_CORE_DllAPI bool TlsConfiguration::is_valid_kind<TlsKind::client>(
        utils::Formatter& error_msg) const noexcept
{
    if (verify_peer)
    {
        if (certificate_authority_file.empty())
        {
            // TODO check it is a correct file
            error_msg << "Invalid certificate_authority_file while server verification must be done.";
            return false;
        }
    }
    return true;
}

template <>
DDSPIPE_CORE_DllAPI bool TlsConfiguration::is_valid_kind<TlsKind::server>(
        utils::Formatter& error_msg) const noexcept
{
    if (private_key_file.empty())
    {
        // TODO check it is a correct file
        error_msg << "Invalid private_key_file.";
        return false;
    }

    if (dh_params_file.empty())
    {
        // TODO check it is a correct file
        error_msg << "Invalid dh_params_file.";
        return false;
    }

    // chain cert is not required, however is usually needed
    if (certificate_chain_file.empty())
    {
        // TODO check it is a correct file
        error_msg << "Invalid certificate_chain_file.";
        return false;
    }

    return true;
}

template <>
DDSPIPE_CORE_DllAPI bool TlsConfiguration::is_valid_kind<TlsKind::both>(
        utils::Formatter& error_msg) const noexcept
{
    return is_valid_kind<TlsKind::client>(error_msg) && is_valid_kind<TlsKind::server>(error_msg);
}

bool TlsConfiguration::is_valid(
        utils::Formatter& error_msg) const noexcept
{
    switch (kind)
    {
        case TlsKind::client:
            return is_valid_kind<TlsKind::client>(error_msg);

        case TlsKind::server:
            return is_valid_kind<TlsKind::server>(error_msg);

        case TlsKind::both:
            return is_valid_kind<TlsKind::both>(error_msg);

        default:
            // None
            return true;
    }
}

} /* namespace types */
} /* namespace participants */
} /* namespace ddspipe */
} /* namespace eprosima */

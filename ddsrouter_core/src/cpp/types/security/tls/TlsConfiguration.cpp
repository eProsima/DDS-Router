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

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/utils.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/exception/ConfigurationException.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <ddsrouter_core/types/security/tls/TlsConfiguration.hpp>

namespace eprosima {
namespace ddsrouter {
namespace core {
namespace types {
namespace security {

template <>
void TlsConfiguration::check_valid_<TlsKind::client>() const
{
    if (certificate_authority_file_.empty())
    {
        // TODO check it is a correct file
        throw utils::InitializationException(utils::Formatter() << "Invalid certificate_authority_file");
    }
}

template <>
void TlsConfiguration::check_valid_<TlsKind::server>() const
{
    // TODO check every file is correct
    if (private_key_file_.empty() || certificate_chain_file_.empty() || dh_params_file_.empty())
    {
        throw utils::InitializationException(utils::Formatter() << "At least one invalid file");
    }
}

template <>
void TlsConfiguration::check_valid_<TlsKind::both>() const
{
    this->check_valid_<TlsKind::client>();
    this->check_valid_<TlsKind::server>();
}

// Inactive constructor
TlsConfiguration::TlsConfiguration()
    : kind_(TlsKind::inactive)
{
}

// Client constructor
TlsConfiguration::TlsConfiguration(
        std::string certificate_authority_file)
    : kind_(TlsKind::client)
    , certificate_authority_file_(certificate_authority_file)
{
    this->check_valid_<TlsKind::client>();
}

// Server constructor
TlsConfiguration::TlsConfiguration(
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_chain_file,
        std::string dh_params_file)
    : kind_(TlsKind::server)
    , private_key_file_password_(private_key_file_password)
    , private_key_file_(private_key_file)
    , certificate_chain_file_(certificate_chain_file)
    , dh_params_file_(dh_params_file)
{
    this->check_valid_<TlsKind::server>();
}

// Server & client constructor
TlsConfiguration::TlsConfiguration(
        std::string certificate_authority_file,
        std::string private_key_file_password,
        std::string private_key_file,
        std::string certificate_chain_file,
        std::string dh_params_file)
    : kind_(TlsKind::both)
    , certificate_authority_file_(certificate_authority_file)
    , private_key_file_password_(private_key_file_password)
    , private_key_file_(private_key_file)
    , certificate_chain_file_(certificate_chain_file)
    , dh_params_file_(dh_params_file)
{
    this->check_valid_<TlsKind::both>();
}

bool TlsConfiguration::is_active() const noexcept
{
    return this->kind_ != TlsKind::inactive;
}

const std::string& TlsConfiguration::certificate_authority_file() const
{
    if (this->compatible<TlsKind::client>())
    {
        return certificate_authority_file_;
    }
    else
    {
        throw utils::InconsistencyException("Cannot get certificate_authority_file: incompatible with TlsKind::client");
    }
}

const std::string& TlsConfiguration::private_key_file_password() const
{
    if (this->compatible<TlsKind::server>())
    {
        return private_key_file_password_;
    }
    else
    {
        throw utils::InconsistencyException("Cannot get private_key_file_password: incompatible with TlsKind::server");
    }
}

const std::string& TlsConfiguration::private_key_file() const
{
    if (this->compatible<TlsKind::server>())
    {
        return private_key_file_;
    }
    else
    {
        throw utils::InconsistencyException("Cannot get private_key_file: incompatible with TlsKind::server");
    }
}

const std::string& TlsConfiguration::certificate_chain_file() const
{
    if (this->compatible<TlsKind::server>())
    {
        return certificate_chain_file_;
    }
    else
    {
        throw utils::InconsistencyException("Cannot get certificate_chain_file: incompatible with TlsKind::server");
    }
}

const std::string& TlsConfiguration::dh_params_file() const
{
    if (this->compatible<TlsKind::server>())
    {
        return dh_params_file_;
    }
    else
    {
        throw utils::InconsistencyException("Cannot get dh_params_file: no compatible with TlsKind::server");
    }
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

    // Perform verification of the server
    descriptor->tls_config.add_verify_mode(
        eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_PEER);

    if (client)
    {
        if (!compatible<types::security::TlsKind::client>())
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
        if (!compatible<types::security::TlsKind::server>())
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
            if (compatible<types::security::TlsKind::client>())
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
    if (!compatible<types::security::TlsKind::client>())
    {
        utils::tsnh(
            utils::Formatter() << "Error, TlsConfiguration expected a Client-compatible configuration.");
    }

    if (only_client)
    {
        // Fail verification if the server has no certificate
        descriptor->tls_config.add_verify_mode(
            eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT);
    }

    // CA certificate
    descriptor->tls_config.verify_file = certificate_authority_file();
}

void TlsConfiguration::enable_tls_server(
        std::shared_ptr<eprosima::fastdds::rtps::TCPTransportDescriptor> descriptor) const
{
    if (!compatible<types::security::TlsKind::server>())
    {
        utils::tsnh(
            utils::Formatter() << "Error, TlsConfiguration expected a Server-compatible configuration.");
    }

    // Password
    descriptor->tls_config.password = private_key_file_password();
    // Private key
    descriptor->tls_config.private_key_file = private_key_file();
    // DDS-Router certificate
    descriptor->tls_config.cert_chain_file = certificate_chain_file();
    // DH
    descriptor->tls_config.tmp_dh_file = dh_params_file();
}

} /* namespace security */
} /* namespace types */
} /* namespace core */
} /* namespace ddsrouter */
} /* namespace eprosima */

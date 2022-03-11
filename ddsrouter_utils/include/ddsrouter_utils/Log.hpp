// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file Log.hpp
 */

#ifndef _DDSROUTERUTILS_LOG_HPP_
#define _DDSROUTERUTILS_LOG_HPP_

// Use FastDDS log
#include <fastdds/dds/log/Log.hpp>

#include <ddsrouter_utils/macros.hpp>

namespace eprosima {
namespace ddsrouter {
namespace utils {

using Log = eprosima::fastdds::dds::Log;
using LogConsumer = eprosima::fastdds::dds::LogConsumer;

/**
 * @brief Log level only for debugging purpose (entities creation and destruction, unexpected behaviour, etc.)
 *
 * As this level is not implemented, it is used as Info level.
 */
#define logDebug(cat, msg) logDebug_(cat, msg)

/**
 * @brief Log level for messages that will be shown to the User (user interactions, start or finish process, etc.)
 *
 * As this level is not implemented, it is used as Info level.
 *
 * @todo Decide if setting the log TAG as in fastrtps logging or not.
 */
#define logUser(cat, msg) std::cout /* << STRINGIFY(cat) << " : " */ << msg << std::endl;

// Allow multiconfig platforms like windows to disable info queueing on Release and other non-debug configs
#if !HAVE_LOG_NO_INFO &&  \
    (defined(FASTDDS_ENFORCE_LOG_INFO) || \
    ((defined(__INTERNALDEBUG) || defined(_INTERNALDEBUG)) && (defined(_DEBUG) || defined(__DEBUG) || \
    !defined(NDEBUG))))
#define logDebug_(cat, msg)                                                                              \
    {                                                                                                   \
        using namespace eprosima::fastdds::dds;                                                         \
        if (Log::GetVerbosity() >= Log::Kind::Info)                                                     \
        {                                                                                               \
            std::stringstream fastdds_log_ss_tmp__;                                                     \
            fastdds_log_ss_tmp__ << "DEBUG: " << msg;                                                    \
            Log::QueueLog(fastdds_log_ss_tmp__.str(), Log::Context{__FILE__, __LINE__, __func__, #cat}, \
                    Log::Kind::Info);                                                                   \
        }                                                                                               \
    }
#elif (__INTERNALDEBUG || _INTERNALDEBUG)
#define logDebug_(cat, msg)                                  \
    {                                                       \
        auto fastdds_log_lambda_tmp__ = [&]()               \
                {                                           \
                    std::stringstream fastdds_log_ss_tmp__; \
                    fastdds_log_ss_tmp__ << "DEBUG: " << msg; \
                };                                          \
        (void)fastdds_log_lambda_tmp__;                     \
    }
#else
#define logDebug_(cat, msg)
#endif // ifndef LOG_NO_INFO

// Allow multiconfig platforms like windows to disable info queueing on Release and other non-debug configs
#if !HAVE_LOG_NO_INFO &&  \
    (defined(FASTDDS_ENFORCE_LOG_INFO) || \
    ((defined(__INTERNALDEBUG) || defined(_INTERNALDEBUG)) && (defined(_DEBUG) || defined(__DEBUG) || \
    !defined(NDEBUG))))
#define logUser_(cat, msg)                                                                              \
    {                                                                                                   \
        using namespace eprosima::fastdds::dds;                                                         \
        if (Log::GetVerbosity() >= Log::Kind::Info)                                                     \
        {                                                                                               \
            std::stringstream fastdds_log_ss_tmp__;                                                     \
            fastdds_log_ss_tmp__ << "USER: " << msg;                                                    \
            Log::QueueLog(fastdds_log_ss_tmp__.str(), Log::Context{__FILE__, __LINE__, __func__, #cat}, \
                    Log::Kind::Info);                                                                   \
        }                                                                                               \
    }
#elif (__INTERNALDEBUG || _INTERNALDEBUG)
#define logUser_(cat, msg)                                  \
    {                                                       \
        auto fastdds_log_lambda_tmp__ = [&]()               \
                {                                           \
                    std::stringstream fastdds_log_ss_tmp__; \
                    fastdds_log_ss_tmp__ << "USER: " << msg; \
                };                                          \
        (void)fastdds_log_lambda_tmp__;                     \
    }
#else
#define logUser_(cat, msg)
#endif // ifndef LOG_NO_INFO

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

#endif /* _DDSROUTERUTILS_LOG_HPP_ */

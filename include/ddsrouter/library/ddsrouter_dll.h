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

#ifndef _EPROSIMA_DDSROUTER_DLL_HPP_
#define _EPROSIMA_DDSROUTER_DLL_HPP_

#include "config.h"

// normalize macros
#if !defined(DDSROUTER_DYN_LINK) && !defined(DDSROUTER_STATIC_LINK) \
    && !defined(EPROSIMA_ALL_DYN_LINK) && !defined(EPROSIMA_ALL_STATIC_LINK)
#define DDSROUTER_STATIC_LINK
#endif // STATIC LINK CHECKS

#if defined(EPROSIMA_ALL_DYN_LINK) && !defined(DDSROUTER_DYN_LINK)
#define DDSROUTER_DYN_LINK
#endif // DYNAMIC LINK CHECKS

#if defined(DDSROUTER_DYN_LINK) && defined(DDSROUTER_STATIC_LINK)
#error Must not define both DDSROUTER_DYN_LINK and DDSROUTER_STATIC_LINK
#endif // DYNAMIC AND STATIC SANITY CHECK

#if defined(EPROSIMA_ALL_NO_LIB) && !defined(DDSROUTER_NO_LIB)
#define DDSROUTER_NO_LIB
#endif // NO_LIB CHECK

// enable dynamic linking

#if defined(_WIN32)
#if defined(EPROSIMA_ALL_DYN_LINK) || defined(DDSROUTER_DYN_LINK)
#if defined(DDSROUTER_SOURCE)
#define DDSROUTER_DllAPI __declspec( dllexport )
#else
#define DDSROUTER_DllAPI __declspec( dllimport )
#endif // DDSROUTER_SOURCE
#else
#define DDSROUTER_DllAPI
#endif // DYNAMIC LINK
#else
#define DDSROUTER_DllAPI
#endif // _WIN32

// enabling user dynamic linking
#if defined(_WIN32) && defined(DDSROUTER_USER_DLL_EXPORT)
  #define DDSROUTER_USERDllExport __declspec(dllexport)
#else
  #define DDSROUTER_USERDllExport
#endif // USER_DLL_EXPORT

// Auto linking.

#if !defined(DDSROUTER_SOURCE) && !defined(EPROSIMA_ALL_NO_LIB) \
    && !defined(DDSROUTER_NO_LIB)

// Set properties.
#define EPROSIMA_LIB_NAME DDSROUTER

#if defined(EPROSIMA_ALL_DYN_LINK) || defined(DDSROUTER_DYN_LINK)
#define EPROSIMA_DYN_LINK
#endif // DYNAMIC LINK

#include "eProsima_auto_link.h"
#endif // auto-linking disabled

#endif // _EPROSIMA_DDSROUTER_DLL_HPP_

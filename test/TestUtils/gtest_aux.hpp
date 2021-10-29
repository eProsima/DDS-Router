// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// This file is part of eProsima Fast DDS Monitor.
//
// eProsima Fast DDS Monitor is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// eProsima Fast DDS Monitor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with eProsima Fast DDS Monitor. If not, see <https://www.gnu.org/licenses/>.

/**
 * @file gtest_aux.hpp
 */

#ifndef _EPROSIMA_FASTDDS_STATISTICS_BACKEND_TEST_GTEST_AUX_HPP_
#define _EPROSIMA_FASTDDS_STATISTICS_BACKEND_TEST_GTEST_AUX_HPP_

#ifdef _WIN32

#include <stdio.h>

int __cdecl chdir(
        char const* _Path);
FILE* __cdecl fdopen(
        int _FileHandle,
        char const* _Format);
int __cdecl read(
        int _FileHandle,
        void* _DstBuf,
        unsigned int _MaxCharCount);
int __cdecl write(
        int _FileHandle,
        void const* _Buf,
        unsigned int _MaxCharCount);
int __cdecl close(
        int _FileHandle);

#endif // _WIN32

#endif // _EPROSIMA_FASTDDS_STATISTICS_BACKEND_TEST_GTEST_AUX_HPP_


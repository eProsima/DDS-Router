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
 * @file Formatter.hpp
 *
 * This file contains class Formatter implementation.
 */

#ifndef _DDSROUTERUTILS_FORMATTET_HPP_
#define _DDSROUTERUTILS_FORMATTET_HPP_

#include <sstream>
#include <string>

#include <ddsrouter_utils/library/library_dll.h>

// Fast use of Formatter call, maybe simpler to the user
#define STR_ENTRY eprosima::ddsrouter::utils::Formatter()

namespace eprosima {
namespace ddsrouter {
namespace utils {

/**
 * @brief Auxiliary function to concatenate inplace every kind of object << stream
 *
 * The main case to use this class is in Exception creation. In order to generate an Exception message
 * using the << operator for the objects in the block, add them to Formatter() object and they will be
 * concatenated in a single string.
 *
 * Example of use:
 * Exception(STR_ENTRY << " object1 stream: " << obj1 << " object2 stream: " << obj2);
 */
class Formatter
{
public:

    //! Concatenate stream values to this formatter
    template<class Val>
    Formatter& operator <<(
            const Val& val);

    //! Cast operator to implicitly cast a \c Formatter to a \c std::string
    DDSROUTER_UTILS_DllAPI operator std::string() const noexcept;

    //! Return a string with the concatenation of this object
    DDSROUTER_UTILS_DllAPI std::string to_string() const noexcept;

protected:

    //! Concatenated stream where the streams are added at the end
    std::stringstream ss_;
};

//! \c Formatter to stream serializator
DDSROUTER_UTILS_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const Formatter& f);

} /* namespace utils */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_utils/impl/Formatter.ipp>

#endif /* _DDSROUTERUTILS_FORMATTET_HPP_ */

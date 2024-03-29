// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

/*!
 * @file HelloWorldKeyed.cpp
 * This source file contains the implementation of the described types in the IDL file.
 *
 * This file was generated by the tool fastddsgen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "HelloWorldKeyed.h"

#if FASTCDR_VERSION_MAJOR > 1

#include <fastcdr/Cdr.h>


#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>




HelloWorldKeyed::HelloWorldKeyed()
{
}

HelloWorldKeyed::~HelloWorldKeyed()
{
}

HelloWorldKeyed::HelloWorldKeyed(
        const HelloWorldKeyed& x)
{
    m_id = x.m_id;
    m_index = x.m_index;
    m_message = x.m_message;
}

HelloWorldKeyed::HelloWorldKeyed(
        HelloWorldKeyed&& x) noexcept
{
    m_id = x.m_id;
    m_index = x.m_index;
    m_message = std::move(x.m_message);
}

HelloWorldKeyed& HelloWorldKeyed::operator =(
        const HelloWorldKeyed& x)
{

    m_id = x.m_id;
    m_index = x.m_index;
    m_message = x.m_message;
    return *this;
}

HelloWorldKeyed& HelloWorldKeyed::operator =(
        HelloWorldKeyed&& x) noexcept
{

    m_id = x.m_id;
    m_index = x.m_index;
    m_message = std::move(x.m_message);
    return *this;
}

bool HelloWorldKeyed::operator ==(
        const HelloWorldKeyed& x) const
{
    return (m_id == x.m_id &&
           m_index == x.m_index &&
           m_message == x.m_message);
}

bool HelloWorldKeyed::operator !=(
        const HelloWorldKeyed& x) const
{
    return !(*this == x);
}

/*!
 * @brief This function sets a value in member id
 * @param _id New value for member id
 */
void HelloWorldKeyed::id(
        int32_t _id)
{
    m_id = _id;
}

/*!
 * @brief This function returns the value of member id
 * @return Value of member id
 */
int32_t HelloWorldKeyed::id() const
{
    return m_id;
}

/*!
 * @brief This function returns a reference to member id
 * @return Reference to member id
 */
int32_t& HelloWorldKeyed::id()
{
    return m_id;
}

/*!
 * @brief This function sets a value in member index
 * @param _index New value for member index
 */
void HelloWorldKeyed::index(
        uint32_t _index)
{
    m_index = _index;
}

/*!
 * @brief This function returns the value of member index
 * @return Value of member index
 */
uint32_t HelloWorldKeyed::index() const
{
    return m_index;
}

/*!
 * @brief This function returns a reference to member index
 * @return Reference to member index
 */
uint32_t& HelloWorldKeyed::index()
{
    return m_index;
}

/*!
 * @brief This function copies the value in member message
 * @param _message New value to be copied in member message
 */
void HelloWorldKeyed::message(
        const std::string& _message)
{
    m_message = _message;
}

/*!
 * @brief This function moves the value in member message
 * @param _message New value to be moved in member message
 */
void HelloWorldKeyed::message(
        std::string&& _message)
{
    m_message = std::move(_message);
}

/*!
 * @brief This function returns a constant reference to member message
 * @return Constant reference to member message
 */
const std::string& HelloWorldKeyed::message() const
{
    return m_message;
}

/*!
 * @brief This function returns a reference to member message
 * @return Reference to member message
 */
std::string& HelloWorldKeyed::message()
{
    return m_message;
}

// Include auxiliary functions like for serializing/deserializing.
#include "HelloWorldKeyedCdrAux.ipp"

#endif // FASTCDR_VERSION_MAJOR > 1

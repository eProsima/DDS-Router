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
 * @file CollectionWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_COLLECTIONWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_COLLECTIONWAITHANDLER_HPP_

#include <ddsrouter_event/wait/CounterWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * \c CollectionWaitHandler is a class that implements a Wait Handler storing data in a collection and allows
 * threads to wait until some data is inserted, and remove it from the collection.
 * The typical use will be to implement this class with an internal queue or stack to store sorted data and
 * consume this data from threads that may wait for a data to be available.
 *
 * \c T specializes this class depending on the data that is stored inside the collection.
 *
 * This class uses an internal CounterWaitHandler to count the number of data stored in the collection.
 *
 * WARNING: This class does not protect the internal collection. Making it thread safe is users duty.
 * WARNING: The collection must be inside this handler object and should not be accessed outside this class methods.
 *
 * TODO: reimplement this class without CounterWaitHandler inheritance
 */
template <typename T>
class CollectionWaitHandler : protected CounterWaitHandler
{
public:

    CollectionWaitHandler(bool enabled = true);

    // Make this parent methods public
    using CounterWaitHandler::enable;
    using CounterWaitHandler::disable;
    using CounterWaitHandler::enabled;
    using CounterWaitHandler::stop_and_continue;

    /////
    // Add values methods

    /**
     * @brief Add a new value to the collection. Use move constructor.
     *
     * This method will awake ONE thread waiting for data to be available if there is any waiting
     * Otherwise it will store the data inside the collection.
     *
     * @note this method calls \c add_value_ , method that must be overriden by the child class.
     *
     * @param value new data available
     */
    void add_value(T&& value);

    //! Add a new value to the collection. Use copy constructor.
    void add_value(const T& value);

    /////
    // Get values methods

    /**
     * @brief Wait until there is data available in the internal collection and retrieve the first one.
     *
     * This method will wait until there is data available in the internal collection.
     * In case there are data (or some data is stored while waiting) the internal collection is the one that decides
     * which data is returned (if FIFO, LIFO, etc. depending on the collection type).
     *
     * @note this method calls \c get_next_value_ , method that must be overriden by the child class.
     *
     * @param timeout maximum time to wait for data in milliseconds. If 0, not time limit. [default 0].
     * @return T next value available in the collection.
     *
     * @throw \c DisabledException if the handler is disabled when calling this method or while waiting.
     * @throw \c TimeoutException if timeout is reached.
     */
    T wait_next_value(
        const utils::Duration_ms& timeout = 0);

protected:

    /**
     * @brief Method that adds a new value in the collection. Use move constructor.
     *
     * This method must be reimplemented in child classes specialized to the internal collection.
     *
     * This method is called without any mutex taken and afterwards the internal counter is increased by 1.
     *
     * @param value new value
     */
    virtual void add_value_(T&& value) = 0;

    //! Method that adds a new value in the collection. Use copy constructor.
    virtual void add_value_(const T& value) = 0;

    /**
     * @brief Method that gets next available value from the collection
     *
     * This method must be reimplemented in child classes specialized to the internal collection.
     * Typically it will remove this value from the collection, but it is not mandatory.
     *
     * @warning This method must assure that values are retrieved whenever \c add_value_
     * has been called more times than \c get_next_value_ .
     *
     * This method is called without any mutex taken and after the internal counter is decreased by 1.
     *
     * @return next value from the collection.
     *
     * @throw \c IncosistencyException if no data is available.
     * This should not happen and is a bug from the child implementation.
     */
    virtual T get_next_value_() = 0;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/wait/impl/CollectionWaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAIT_COLLECTIONWAITHANDLER_HPP_ */

.. include:: ../exports/alias.include
.. include:: ../exports/roles.include

.. _user_manual_wan_configuration:

#################
WAN Configuration
#################

In order to communicate a |ddsrouter| via :term:`WAN`, some configurations may be required.


NAT Traversal
=============

If the |ddsrouter| is under a :term:`NAT`, a remote |ddsrouter| in a different :term:`LAN` will not be able to
reach it.
Thus, NAT traversal methods will be required.
The most common method that we recommend is configuring the network router so it forwards a specific port from
the internet to a specific host.


Port Forwarding
---------------

This is the easiest way to achieve NAT traversal.
Most network routers support a graphical interface where port forwarding could be easily set.


.. note:

    NAT Traversal communication only affects to IPv4 communication.
    Using IPv6 would not require specific network router configurations (as long as the router supports it).

.. note:

    It is needed to use same public port and internal port due to of Fast DDS configuration requirements.


TCP vs UDP
==========

:term:`TCP` and :term:`UDP` are two well known network transport protocols.
Both has its advantages and disadvantages regarding the scenario.
These are a list of tips to help choosing whether to use one or the other.

.. list-table::
    :header-rows: 1

    *   - Feature
        - UDP
        - TCP

    *   - **Communication** |br|
          **speed**
        - Fast
        - Slower

    *   - **Reliability**
        - No Transport Layer reliability |br|
          (could has DDS reliability)
        - Transport Layer reliability |br|
          (duplicated if DDS reliability is used)

    *   - **Port Forwarding**
        - Require both sides of the communication |br|
          to have ports forwarded from the router.
        - Require only server side of the communication |br|
          to have port forwarded from the router.

.. note:

    DDS is thought to work over UDP and has its own reliability mechanisms.
    Thus, the |ddsrouter| uses UDP transport by default for every address that has not explicitly specified
    a transport in the configuration file.


Examples
========

TCP Port Forwarding Example
---------------------------

Let be the scenario where user *A* host *H*:sub:`A` has a private IP ``192.168.1.2`` given by network router
*R*:sub:`A`, with a public IP ``1.1.1.1``.
Let user *B* with host *H*:sub:`B` has a private IP ``192.168.2.2`` given by network router *R*:sub:`B`,
with a public IP ``2.2.2.2``.
*A* will act as server of the TCP communication, while *B* will act as client.

User *A* should set a port forwarding rule in router *R*:sub:`A` as ``11666 -> 192.168.1.2:11666``.
That is, every datagram that arrives to IP ``1.1.1.1:11666`` will be forwarded to ``192.168.1.2:11666``
(it is required to use the same public port as the internal one).
User *A* should set its *listening-addresses* as follows:

.. yaml:

    id: 2                       # Id to generate the GuidPrefix of the Discovery Server of A
    listening-addresses:
    [
        {
            IP: "1.1.1.1",      # Public IP of host Ha
            port: 11666,        # Port forwarded router Ra
        }
    ]

User *B* should set *connection-addresses* to connect to *H*:sub:`A` as follows:

.. yaml:

    id: 3                               # Must be different than A one
    connection-addresses:
    [
        {
            id: 2                       # Id of the Discovery Server of A
            addresses:
            [
                {
                    IP: "1.1.1.1",      # Public IP of Ha
                    port: 11666,        # Port forwarded in Ra
                }
            ]
        }
    ]

This way, *B* will connect to *A*.
*A* will be able to receive the message because *R*:sub:`A` will forward the message to *H*:sub:`A`.
Once *A* has received the message, a TCP channel will be set, and the communication will travel both ways without
requiring to traverse any other NAT.


UDP Port Forwarding Example
---------------------------

Let be the scenario where user *A* host *H*:sub:`A` has a private IP ``192.168.1.2`` given by network router
*R*:sub:`A`, with a public IP ``1.1.1.1``.
Let user *B* with host *H*:sub:`B` has a private IP ``192.168.2.2`` given by network router *R*:sub:`B`,
with a public IP ``2.2.2.2``.
*A* and *B* will communicate via UDP, so there is no need to set a client and a server.
It does not matter whether *A* knows *B* address, *B* knows *A*, or both know each other.
In this example, *B* will know *A* address, and not the other way around.

User *A* should set a port forwarding rule in router *R*:sub:`A` as ``11666 -> 192.168.1.2:11666``.
That is, every datagram that arrives to IP ``1.1.1.1:11666`` will be forwarded to ``192.168.1.2:11666``
(it is required to use same public port as the internal one).
User *A* should set its *listening-addresses* as follows:

.. yaml:

    id: 2                       # Id to generate the GuidPrefix of the Discovery Server of A
    listening-addresses:
    [
        {
            IP: "1.1.1.1",      # Public IP of host Ha
            port: 11666,        # Port forwarded router Ra
        }
    ]

User *B* should set a port forwarding rule in router *R*:sub:`B` as ``11777 -> 192.168.2.2:11777``.
This is, every datagram that arrives to IP ``2.2.2.2:11777`` will be forwarded to ``192.168.2.2:11777``
(It is necessary to use same public port as the internal one).
User *B* should set its *listening-addresses* and *connection-addresses* as follows:

.. yaml:

    id: 3                               # Must be different than A one
    listening-addresses:
    [
        {
            IP: "2.2.2.2",      # Public IP of host Hb
            port: 11777,        # Port forwarded router Rb
        }
    ]
    connection-addresses:
    [
        {
            id: 2                       # Id of the Discovery Server of A
            addresses:
            [
                {
                    IP: "1.1.1.1",      # Public IP of Ha
                    port: 11666,        # Port forwarded in Ra
                }
            ]
        }
    ]

This way, *B* will connect to *A*.
Once *A* receives the message from *B*, it will communicate with it via address ``2.2.2.2:11777``.
*B* will continue communicating with *A* via address ``1.1.1.1:11666``.

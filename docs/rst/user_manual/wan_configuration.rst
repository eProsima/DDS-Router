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

.. note::

    NAT Traversal communication only affects to IPv4 communication.
    Using IPv6 would not create NAT under network routers so every device could be accessed externally.
    Thus, configurations explained in this section does not apply to IPv6 deployments.


.. _user_manual_wan_configuration_nat_traversal_port_forwarding:

Port Forwarding
---------------

This is the easiest way to achieve NAT traversal.
Most network routers support a graphical interface where port forwarding could be easily set.


TCP vs UDP
==========

:term:`TCP` and :term:`UDP` are two well known network transport protocols.
Both have their advantages and disadvantages regarding the scenario.
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

.. note::

    DDS is thought to work over UDP and has its own reliability mechanisms.
    Thus, the |ddsrouter| uses UDP transport by default for every address that has not explicitly specified
    a transport in the configuration file.


TLS
===

|eddsrouter| also supports `TLS over TCP <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/tcp/tls.html>`_,
and its configuration can be set per participant for types Local Discovery Server and WAN. Following is a list of the
accepted entries under the ``tls`` tag:

.. list-table::
    :header-rows: 1

    *   - Tag
        - Requiredness
        - Description

    *   - ``ca``
        - Mandatory for TLS servers and clients
        - Path to the CA (Certification- Authority) file.

    *   - ``password``
        - Optional for TLS servers
        - Password of the ``private_key`` file.

    *   - ``private_key``
        - Mandatory for TLS servers
        - Path to the private key certificate file.

    *   - ``cert``
        - Mandatory for TLS servers
        - Path to the public certificate chain file.

    *   - ``dh_params``
        - Mandatory for TLS servers
        - Path to the Diffie-Hellman parameters file.

.. note::

    Although in principle only required for TLS clients, the CA (Certification- Authority) file must also be provided
    for TLS servers, as they might assume the client role when connecting to other participants configured as servers.


Examples
========

.. _tcp_example:

TCP Port Forwarding Example
---------------------------

Let be the scenario where user *A* host *H*:sub:`A` has a private IP ``192.168.1.2`` given by network router
*R*:sub:`A`, with a public IP ``1.1.1.1``.
Let user *B* with host *H*:sub:`B` has a private IP ``192.168.2.2`` given by network router *R*:sub:`B`,
with a public IP ``2.2.2.2``.
*A* will act as server of the TCP communication, while *B* will act as client.

User *A* should set a port forwarding rule in router *R*:sub:`A` as ``11666 -> 192.168.1.2:11667``.
That is, every datagram that arrives to IP ``1.1.1.1:11666`` will be forwarded to ``192.168.1.2:11667``.
User *A* should set its *listening-addresses* as follows:

.. code-block:: yaml

    - name: WANServerParticipant_userA
      kind: wan

      discovery-server-guid:
        id: 2                             # Id to generate the GuidPrefix of the Discovery Server of A
      listening-addresses:
        - ip: 1.1.1.1                     # Public IP of host Ha
          port: 11667                     # Physical port used for the dds router host
          external-port: 11666            # Port forwarded router Ra
          transport: tcp                  # Transport protocol

User *B* should set *connection-addresses* to connect to *H*:sub:`A` as follows:

.. code-block:: yaml

    - name: WANClientParticipant_userB
      kind: wan

      discovery-server-guid:
        id: 3                             # Must be different than A one
      connection-addresses:
        - discovery-server-guid:
            id: 2                         # Id of the Discovery Server of A
          addresses:
            - ip: 1.1.1.1                 # Public IP of Ha
              port: 11666                 # Port forwarded in Ra
              transport: tcp              # Transport protocol

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

User *A* should set a port forwarding rule in router *R*:sub:`A` as ``11666 -> 192.168.1.2:11667``.
That is, every datagram that arrives to IP ``1.1.1.1:11666`` will be forwarded to ``192.168.1.2:11667``.
User *A* should set its *listening-addresses* as follows:

.. code-block:: yaml

    - name: WANServerParticipant_userA
      kind: wan

      discovery-server-guid:
        id: 2                             # Id to generate the GuidPrefix of the Discovery Server of A
      listening-addresses:
        - ip: 1.1.1.1                     # Public IP of host Ha
          port: 11667                     # This device physical port forwarded internally
          external-port: 11666            # Port forwarded router Ra

User *B* should set a port forwarding rule in router *R*:sub:`B` as ``11777 -> 192.168.2.2:11778``.
This is, every datagram that arrives to IP ``2.2.2.2:11777`` will be forwarded to ``192.168.2.2:11778``.
User *B* should set its *listening-addresses* and *connection-addresses* as follows:

.. code-block:: yaml

    - name: WANClientParticipant_userB
      kind: wan

      discovery-server-guid:
        id: 3                             # Must be different than A one
      listening-addresses:
        - ip: 2.2.2.2                     # Public IP of host Hb
          port: 11778                     # This device physical port forwarded internally
          external-port: 11777            # Port forwarded router Rb
      connection-addresses:
        - discovery-server-guid:
            id: 2                         # Id of the Discovery Server of A
          addresses:
            - ip: 1.1.1.1                 # Public IP of Ha
              port: 11666                 # Port forwarded in Ra

This way, *B* will connect to *A*.
Once *A* receives the message from *B*, it will communicate with it via address ``2.2.2.2:11777``.
*B* will continue communicating with *A* via address ``1.1.1.1:11666``.


TLS Configuration Example
-------------------------

Below is an example on how to configure a WAN participant as a TLS server and client:

.. code-block:: yaml

    - name: TLS_Server
      kind: wan

      discovery-server-guid:
        id: 0
      listening-addresses:
        - ip: 1.1.1.1
          port: 11666
          transport: tcp

      tls:
        ca: ca.crt
        password: ddsrouterpass
        private_key: ddsrouter.key
        cert: ddsrouter.crt
        dh_params: dh_params.pem

.. code-block:: yaml

    - name: TLS_Client
      kind: wan

      discovery-server-guid:
        id: 1
      connection-addresses:
        - discovery-server-guid:
            id: 0
          addresses:
            - ip: 1.1.1.1
              port: 11666
              transport: tcp

      tls:
        ca: ca.crt

You may also have a look at ``<path/to/ddsrouter_tool>/share/resources/configurations/security/`` directory, which
contains examples of key and certificate files as well as a script with the commands used to generate them.

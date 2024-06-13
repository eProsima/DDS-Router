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
the Internet to a specific host.

.. note::

    NAT Traversal communication only affects to IPv4 communication.
    Using IPv6 would not create NAT under network routers so every device could be accessed externally.
    Thus, configurations explained in this section do not apply to IPv6 deployments.


.. _user_manual_wan_configuration_nat_traversal_port_forwarding:

Port Forwarding
---------------

This is the easiest way to achieve NAT traversal.
Most network routers support a graphical interface where port forwarding could be easily set.


.. _user_manual_wan_configuration_nat_traversal_port_forwarding_external_port:

External port
^^^^^^^^^^^^^

In order to configure a |ddsrouter| Server that runs under a NAT and uses TCP transport, two ports must be
taken into account. The internal port (a.k.a. ``port``) is the one that the host of the |ddsrouter| will use
to open a socket and to receive information.
The external port (:code:`external-port`) references the public port meant for other entities to be able
to locate this |ddsrouter|.
Setting the external port is useful so the network router port forwarding could redirect from a public port
to a different value of internal host port.
Client's port must match the server's external port unless server's external port is not set, in which case it
should match the server's port.

.. note::

    External port configuration is not mandatory. If not set, the internal and the external port must coincide
    in the network router port forwarding rules.

.. warning::

    External port is only available for TCP communication.
    In UDP communication the internal and the external port must coincide in the network router port forwarding rules.


TCP vs UDP
==========

:term:`TCP` and :term:`UDP` are two well known network transport protocols.
Both have their advantages and disadvantages depending on the scenario.
The following comparison is meant to help users choose between one or the other.

.. list-table::
    :header-rows: 1

    *   - Feature
        - UDP
        - TCP

    *   - **Communication** |br|
          **speed**
        - Faster.
        - Slower.

    *   - **Reliability**
        - No Transport Layer reliability |br|
          (but it can have DDS reliability).
        - Transport Layer reliability |br|
          (duplicated if DDS reliability is enabled).

    *   - **Port Forwarding**
        - Both sides of the communication must have |br|
          their ports forwarded from the router. |br|
          Internal and external ports must coincide.
        - Only the server side of the communication |br|
          must have its ports forwarded from the router.

.. note::

    DDS is thought to work over UDP and has its own reliability mechanisms.
    Thus, the |ddsrouter| uses UDP transport by default for every address that has not explicitly specified
    a transport in the configuration file.


TLS
===

|eddsrouter| also supports `TLS over TCP <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/tcp/tls.html>`_,
and its configuration can be set per participant for types WAN Discovery Server and WAN. The following is a list of the
accepted entries under the ``tls`` tag:

.. list-table::
    :header-rows: 1

    *   - Tag
        - Requirements
        - Description
        - Example

    *   - ``ca``
        - Mandatory for TLS clients if |br|
          ``peer_verification`` is active.
        - Path to the CA (Certification- Authority) file.
        - ``ca.crt``

    *   - ``password``
        - Optional for TLS servers.
        - Password of the ``private_key`` file.
        - ``<private_key_file_password>``

    *   - ``private_key``
        - Mandatory for TLS servers.
        - Path to the private key certificate file.
        - ``ddsrouter.key``

    *   - ``cert``
        - Mandatory for TLS servers.
        - Path to the public certificate chain file.
        - ``ddsrouter.crt``

    *   - ``dh_params``
        - Mandatory for TLS servers.
        - Path to the Diffie-Hellman parameters file.
        - ``dh_params.pem``

    *   - ``peer_verification``
        - Optional for clients.
        - Whether to verify the server. (Default true).
        - ``true``

    *   - ``sni_host``
        - Optional for clients if using SNI.
        - Name of the server to connect with.
        - ``my_server.com``

.. note::

    Although in principle only required for TLS clients (with peer verification),
    the CA (Certification- Authority) file may also be provided
    for TLS servers when willing to connect them to other participants configured as servers.

Examples
========

.. _tcp_example:

TCP Port Forwarding Example
---------------------------

Let there be a scenario where user *A* with host *H*:sub:`A` has a private IP ``192.168.1.2`` given by the network router
*R*:sub:`A` with public IP ``1.1.1.1``.
Let user *B* with host *H*:sub:`B` have a private IP ``192.168.2.2`` given by the network router *R*:sub:`B`
with public IP ``2.2.2.2``.
*A* will act as the server of the TCP communication and *B* will act as the client.

User *A* should set a port forwarding rule in router *R*:sub:`A` as ``11666 -> 192.168.1.2:11667``.
That is, every datagram that arrives to IP ``1.1.1.1:11666`` will be forwarded to ``192.168.1.2:11667``.
User *A* should set its *listening-addresses* as follows:

.. code-block:: yaml

    - name: WANServerParticipant_userA
      kind: wan

      listening-addresses:
        - ip: 1.1.1.1                     # Public IP of host Ha
          port: 11667                     # Physical port used for the dds router host
          external-port: 11666            # Port forwarded router Ra
          transport: tcp                  # Transport protocol

User *B* should set *connection-addresses* to connect to *H*:sub:`A` as follows:

.. code-block:: yaml

    - name: WANClientParticipant_userB
      kind: wan

      connection-addresses:
        - ip: 1.1.1.1                     # Public IP of Ha
          port: 11666                     # Port forwarded in Ra
          transport: tcp                  # Transport protocol

This way, *B* will connect to *A*.
*A* will be able to receive the message because *R*:sub:`A` will forward the message to *H*:sub:`A`.
Once *A* has received the message, a TCP channel will be set, and the communication will travel both ways without
requiring to traverse any other NAT.


UDP Port Forwarding Example
---------------------------

Let there be a scenario where user *A* with host *H*:sub:`A` has a private IP ``192.168.1.2`` given by the network router
*R*:sub:`A` with public IP ``1.1.1.1``.
Let user *B* with host *H*:sub:`B` have a private IP ``192.168.2.2`` given by the network router *R*:sub:`B`
with public IP ``2.2.2.2``.
*A* and *B* will communicate via UDP, so there is no need to set a client and a server.
It does not matter whether *A* knows *B* address, *B* knows *A*, or both know each other.
In this example, *B* will know *A* address, and not the other way around.

User *A* should set a port forwarding rule in router *R*:sub:`A` as ``11666 -> 192.168.1.2:11666``.
That is, every datagram that arrives to IP ``1.1.1.1:11666`` will be forwarded to ``192.168.1.2:11666``.
User *A* should set its *listening-addresses* as follows:

.. code-block:: yaml

    - name: WANServerParticipant_userA
      kind: wan

      listening-addresses:
        - ip: 1.1.1.1                     # Public IP of host Ha
          port: 11666                     # Internal and External port

User *B* should set a port forwarding rule in router *R*:sub:`B` as ``11777 -> 192.168.2.2:11777``.
This is, every datagram that arrives to IP ``2.2.2.2:11777`` will be forwarded to ``192.168.2.2:11777``.
User *B* should set its *listening-addresses* and *connection-addresses* as follows:

.. code-block:: yaml

    - name: WANClientParticipant_userB
      kind: wan

      listening-addresses:
        - ip: 2.2.2.2                     # Public IP of host Hb
          port: 11777                     # Internal and External port
      connection-addresses:
        - ip: 1.1.1.1                     # Public IP of Ha
          port: 11666                     # Port forwarded in Ra

This way, *B* will connect to *A*.
Once *A* receives the message from *B*, it will communicate with it via address ``2.2.2.2:11777``.
*B* will continue communicating with *A* via address ``1.1.1.1:11666``.


TLS Configuration Example
-------------------------

Below is an example on how to configure a WAN participant as a TLS server and client:

.. code-block:: yaml

    - name: TLS_Server
      kind: wan

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

      connection-addresses:
        - ip: 1.1.1.1
          port: 11666
          transport: tcp

      tls:
        ca: ca.crt

You may also have a look at the ``<path/to/ddsrouter_tool>/share/resources/configurations/security/`` directory, which
contains examples of key and certificate files as well as a script with the commands used to generate them.

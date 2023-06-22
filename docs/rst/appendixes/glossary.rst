.. include:: ../exports/alias.include

.. _glossary:

########
Glossary
########

.. glossary::

    LAN
        **Local Area Network**

    NAT
        **Network Address Translation**:
        Typically an internet router multiplexes all the traffic through
        a public IP to several private IPs.
        Usually, the machines under the router network cannot be accessed from the outside unless a Port is forwarded
        in the router configuration, or if such host has previously started a TCP communication with the message source.

    QoS
        **Quality of Service**:
        Collection of attributes and settings that configure the behavior of DDS communications.

    QoS Profile
        Specific QoS that is related with a name, called *profile*.
        This is used to create several entities with the same QoS, or to specify which already established QoS an entity should use.

    RPC
        **Remote Procedural Call**:
        Client-Server protocol that can be run over DDS.
        ROS2 services and actions use this kind of protocol.

    TCP
        **Transmission Control Protocol**

    TURN
        **Traversal Using Relays around NAT**
        A TURN server is a network element that supports a common point of redirection of messages.
        It redirect messages between N nodes connected to it.
        This is a very common technique in order to traverse a NAT or an unreachable network.

    UDP
        **User Datagram Protocol**

    WAN
        **Wide Area Network**

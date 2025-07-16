.. include:: ../exports/alias.include

.. _use_case_repeater:

###################
Repeater DDS Router
###################

A |ddsrouter| could work as a :term:`TURN` **Repeater**.
This means that a |ddsrouter| can be used to repeat messages between networks.

Use case
========

The use of a :term:`TURN` Server is very useful in the following scenarios:

- **NAT Traversal**: If the edge devices are under different NATs, they cannot access each other if no
  ports are opened in their respective internet access points.
- **Unreachable Network**: If edge devices work under different networks (e.g. using different transport protocols,
  connected in different private networks, etc.) cannot reach each other.

The following figure exemplifies these use cases.
When the communication between edge routers is not possible, a Repeater can be set in the middle to forward data
and make the communication possible.

.. figure:: /rst/figures/repeater.png


How to configure
================

This |ddsrouter| configuration is very simple, as all it needs is a :ref:`WAN <user_manual_participants_wan>` or
:ref:`XML <user_manual_participants_xml>` Participant with the extra configuration tag `repeater: true`.
There could be more Participants in this |ddsrouter| and topic filtering is also available.
The Repeater Participant works as any other Participant,
with the particularity that it is able to resend the data it receives.

In order to see an example of this configuration, access to the following example :ref:`example_repeater`.

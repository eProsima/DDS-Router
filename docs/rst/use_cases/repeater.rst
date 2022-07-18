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

- **:term:`NAT` Traversal**: If the edge devices are under different NATs, they cannot access each other if no
  ports are opened in their respective internet access points.
- **Unreachable Network**: If edge devices work under different networks (e.g. using different transport protocols,
  connected in different private networks, etc.) cannot reach each other.

The following figure exemplifies these use cases.
When the communication between edge routers is not possible, a Repeater can be set in the middle to forward data
and make the communication possible.

.. figure:: /rst/figures/repeater.png


How to configure
================

In order to know how to configure this kind of |ddsrouter|, access to the following example :ref:`example_repeater`.

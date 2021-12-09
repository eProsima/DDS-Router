.. include:: ../../exports/alias.include

.. _user_manual_participants_simple:

##################
Simple Participant
##################

This type of :term:`Participant` refers to a Simple DDS :term:`DomainParticipant`.
This Participant will discover all Participants in its own network in the same domain via multicast,
and will communicate with those that share publication or subscription topics.


Use case
========

Use this Participant in order to communicate an internal standard DDS network, such as a ROS2 or Fast DDS network
in the same LAN.


Type aliases
============

* ``simple``
* ``local``


Configuration
=============

Simple Participant allow configure the :term:`Domain Id`.
Check :ref:`Configuration section <user_manual_configuration_domain_id>` for further details.


Configuration Example
=====================

.. code-block:: yaml

    simple_participant:         # Participant Id = simple_participant

        type: simple

        domain: 2               # Domain Id = 2

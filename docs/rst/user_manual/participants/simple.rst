.. include:: ../../exports/alias.include

.. _user_manual_participants_simple:

##################
Simple Participant
##################

This kind of :term:`Participant` refers to a Simple DDS :term:`DomainParticipant`.
This Participant will discover all Participants deployed in its own local network in the same domain via multicast
communication, and will communicate with those that share publication or subscription topics.


Use case
========

Use this Participant in order to communicate an internal standard DDS network, such as a ROS 2 or Fast DDS network
in the same LAN.


Kind aliases
============

* ``simple``
* ``local``


Configuration
=============

The only configuration required to start a Simple Participant is the :term:`Domain Id`
on which it will listen for DDS communications.
Check :ref:`Configuration section <user_manual_configuration_domain_id>` for further details.


Configuration Example
=====================

.. code-block:: yaml

    - name: simple_participant     # Participant Name = simple_participant
      kind: simple
      domain: 2                    # Domain Id = 2

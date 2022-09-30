# DDS Test WAN

Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
This is accomplished by connecting two (`DiscoveryServer` / `InitialPeers`) WAN Participants belonging to different DDS Router instances.
These router instances communicate with the DDS Participants through Simple Participants deployed at those domains.
Note that, although data transmission is accomplished by connecting two WAN Participants, the resulting communication
is still local (i.e. WAN communication is not tested here).

# DDS Test Local

Test communication between two DDS Participants hosted in the same device, but which are at different DDS domains.
This is accomplished by using a DDS Router instance with a Simple Participant deployed at each domain.
First, router initialization is tested, and then communication between the DDS Participants is tested both for topics
*HelloWorld* and *HelloWorldKeyed*.

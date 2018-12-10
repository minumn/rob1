
Introduction
========
This repository holds the opencv example which detects rectangles of a certain
area and color. The script is based on various examples found on opencv webpage

Recommended reads

http://docs.opencv.org/

http://docs.opencv.org/doc/tutorials/imgproc/opening_closing_hats/opening_closing_hats.html

http://docs.opencv.org/doc/tutorials/imgproc/table_of_content_imgproc/table_of_content_imgproc.html



Configuring the webcam
==============
The default IP address of the webcam is 192.168.0.20
with the username being **admin** and there is no password.

Configure the webcam such that access control is disabled for single image fetching. (TODO: explain better),
and increase the resolution to 640x480

configure your virtual machine such that it can access the ethernet and set a static ip address in ubuntu.
When configuring the bridge network please select only the ethernet adapter by clicking the configure adapters button 
in the VM manager.

In ubuntu configure a static ip for eth0 using the gui. the address can be 192.168.0.21 with the netmask set to 255.255.255.0 and empty gateway. click save, and then reselect the connection in the network manager icon. 



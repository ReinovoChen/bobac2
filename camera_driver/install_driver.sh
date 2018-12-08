#!/bin/bash
#sudo
test "$(whoami)" != 'root' && echo "permission denied" && exit 1
#install openni2
dpkg -s libopenni2-dev &>/dev/null || apt-get install libopenni2-dev -y
install -T libOpenNI2.so /usr/lib/libOpenNI2.so && echo "openni2 installed"
#install camera driver
install -t /usr/lib/OpenNI2/Drivers Drivers/* && echo "orbbec camera driver installed"
##install test demo
install -T NiViewer /usr/bin/NiViewer && echo "NiViewer installed"
install -T 56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules && echo "set camera udev rules"
install -T 40-serial.rules /etc/udev/rules.d/40-serial.rules && echo "set serial udev rules"
echo "Bobac will reboot 5 seconds later. And you can test, enjoy!"
sleep 5
reboot

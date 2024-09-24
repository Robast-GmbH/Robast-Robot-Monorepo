If no robast folder appears in /dev when plugging in the NFC or USBtin, 
please copy the .rules files to /etc/udev/rules.d/.
(sudo cp Desktop/70-snap.robast.{}.rules /etc/udev/rules.d/)

+ udev neustarten: 
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

Probably reboot


Mehr info unter: https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Udev%20Rules.html
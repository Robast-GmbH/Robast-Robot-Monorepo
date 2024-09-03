Falls beim einstecken des NFC oder USBtin in dev kein robast ordner erscheint 
bitte die .rules dateien nach /etc/udev/rules.d/ kopieren.
(sudo cp Desktop/70-snap.robast.{}.rules /etc/udev/rules.d/)

und udev neustarten: 
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

warscheinlich müssen beide geräte neu eingesteckt werden before sie erkannt werden.


Mehr info unter: https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Udev%20Rules.html
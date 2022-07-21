#!/bin/bash
nfc=$(find /dev/serial/by-id/ -name 'usb-OEM_TWN4_B1.09_NCF4.06_PRS1.04*')
can=$(find /dev/serial/by-id/ -name 'usb-Microchip_Technology__Inc._USBtin_*')


ln -f $can '/dev/serial/by-id/usb-Robast_Drawer_Serial_To_CAN' 
ln -f $nfc  '/dev/serial/by-id/usb-Robast_Authentication_NFC_Side_Reader' 
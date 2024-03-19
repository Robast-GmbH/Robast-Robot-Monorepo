from crccheck.crc import Crc8

'''
Was brauche ich hier:
- einen converter  von ascii auf byte und andersrum
- List of defined command parts:
    - STX = 0x02
    - n = position of crc
    - Ack/Nak = 0x06 / 0x15
    - CRC
    - ETX = 0x03
- 2,3 hardcoded commands. e.g.:
    - Response Tag Present	STX	n	Source 	22	Ack/Nak	Status Tag	LSB LF ID MSB		LSB Tag ID MSB				LSB HF ID MSB		RSSI X	RSSI Y	RSSI Z	RSSI Abstand	CRC	ETX
        - Set Tag Actor: 0x02 0x10 0x00 0x2E 0x06 0x00035976 0x80 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x3c 0x03 
            also: 0x0210002E060003597680000000000000003c03 -> as ascii: .Yv< oder (base64) b'\x02\x10\x00\x2E\x06\x00\x03\x59\x76\x80\x00\x00\x00\x00\x00\x00\x00\x3c\x03' = AhAALgYAA1l2gAAAAAAAAAA8Aw==
    - Set Tag Actor	STX	n	Target	2E	Ack/Nak	LSB Tag ID MSB				Aktor	tbd	tbd	tbd	tbd	tbd	tbd	tbd	CRC	ETX	
import base64

data = b'\x02\x10\x00\x2E\x06\x00\x03\x59\x76\x80\x00\x00\x00\x00\x00\x00\x00\x3c\x03'
base64_data = base64.b64encode(data)
print(base64_data)
original_data = base64.b64decode(base64_data)
print(original_data)

Ausgaben aus dem pss2000 debug:
02 0D 22 06 2E D0 07 AA 59 03 00 D0 07 D0 03

>  00 02 03 26 FE 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
<-  02 0C 26 06 00 52 00 1E 00 1B 00 05 72 03

'''
class command:
    def __init__(self):
        self.STX = 0x02
        self.n = 0
        self.Ack = 0x06
        self.Nak = 0x15
        self.CRC = 0x00
        self.ETX = 0x03
        self.unused = 0x00
        self.LSBTagIDMSB = 0x00000000
        self.StatusTag = 0x00
        
    def set_tag_actor(self, tag_id = 0x000359AA, actor = 0x80):
        n = 0x10
        command_code = 0x2E
        self.LSBTagIDMSB = tag_id
        if len(self.tag_id) != 8 or not self.tag_id.isalnum():
            raise ValueError("Tag ID must be 4 bytes in hexadecimal format")
        self.actor = actor
        command = [
            self.STX,
            n,
            self.unused,
            command_code,
            self.LSBTagIDMSB,
            self.actor,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.CRC,
            self.ETX]
        return command
        
#noch nicht sicher wie ichs genau implemente aber brauche eig das "Request Tag recieved" und "Set Tag Actor" command evtl noch den Heartbeat evtl auch noch Reader Status fürs debuggen
        
    def convert_to_ascii(self, data):
        pass
    def convert_to_byte(self, data):
        pass
    
def crc_8(buf, data_byte):
  """
  Berechnet die 8-Bit-CRC-Prüfsumme für ein Datenfeld.

  Args:
    buf: Ein Byte-Array mit den Daten.
    data_byte: Die Länge des Datenfeldes in Bytes.

  Returns:
    Die berechnete CRC-8-Prüfsumme.
  """

  crc = 0x00
  data_bit = 0x80

  while data_byte > 0:
    if ((crc & 0x01) != 0) != ((buf[data_byte] & data_bit) != 0):
      # Datenbit EXOR mit Rückkopplung
      crc >>= 1
      crc ^= 0xCD
    else:
      crc >>= 1

    data_bit >>= 1

    if not data_bit:
      data_bit = 0x80
      data_byte -= 1

  return crc
#             0x22, 0x06, 0x2E, 0xD0, 0x07, 0xAA, 0x59, 0x03, 0x00, 0xD0, 0x07
# Beispiel 22 06 2E D0 07 AA 59 03 00 D0 07
# really recieved: 0x0d 0x22 0x06 0x26 0xd0 0x07 0xaa 0x59 0x03 0x00 0xd0 0x07 with crc=0xaf

buf = [0x22, 0x06, 0x26, 0xD0, 0x07, 0xAA, 0x59, 0x03, 0x00, 0xD0, 0x07]
data_byte = len(buf)-1
crc = crc_8(buf, data_byte)
print(f"CRC-8: {hex(crc)}")
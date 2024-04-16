# from crccheck.crc import Crc8
import numpy as np

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
<- 02 0C 26 06 00 52 00 1E 00 1B 00 05 72 03

'''
class DatagramFactory:
    def __init__(self):
        self.STX = "02"
        self.Ack = "06"
        self.Nak = "15"
        self.CRC = "00"
        self.unused = "00"
        
    def calculate_and_add_crc(self, command):
      temporarry_string = transform_array_to_string(command[:-1])
      crc = crc_8(transform_string_to_array(temporarry_string))
      command[-1] = format(crc, '02X')  # Convert CRC to hexadecimal format
      return command
    
    def set_tag_actor(self, hex_tag_id = "000359AA", actor = "01"):
        n = "0F"
        command_code = "2E"
        if len(hex_tag_id) != 8 or not hex_tag_id.isalnum():
            raise ValueError("Tag ID must be 4 bytes in hexadecimal format")
        LSBTagIDMSB = invert_byte_order(hex_tag_id)
        command = [
            self.STX,
            n,
            command_code,
            LSBTagIDMSB,
            actor,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.unused,
            self.CRC]
        command = self.calculate_and_add_crc(command)
        return transform_array_to_string(command[1:])
    
    def get_software_version(self):
        n = "03"
        command_code = "27"
        command = [
            self.STX,
            n,
            command_code,
            self.CRC]
        command = self.calculate_and_add_crc(command)
        return transform_array_to_string(command[1:])
      
    def get_status(self):
        n = "03"
        command_code = "26"
        command = [
            self.STX,
            n,
            command_code,
            self.CRC]
        command = self.calculate_and_add_crc(command)
        return transform_array_to_string(command[1:])

    def get_command(self, message) -> str:
        command = message[2:4]        
        return command
      
    def is_ack(self, message) -> bool:
        ack_nak = message[4:6]
        if ack_nak == self.Ack:
          return True
        else:
          return False
    
    def translate_command(self, message, length):
      crc_position = message[0:2]
      message = message[2:]
      command = message[0:2]
      if message[2:4] != "06":
        return "Nak"
      match command:
        case "22":
          tag_status = message[4:6]
          LSB_LF_ID_MSB = message[6:10]
          LSB_LF_ID_MSB = invert_byte_order(LSB_LF_ID_MSB)
          LSB_Tag_ID_MSB = message[10:18]
          LSB_Tag_ID_MSB = invert_byte_order(LSB_Tag_ID_MSB)
          LSB_HF_ID_MSB = message[18:22]
          LSB_HF_ID_MSB = invert_byte_order(LSB_HF_ID_MSB)
        case "26":
          output = message[4:6]
          modis = message[6:8]
          input = message[8:10]
          lf_distance = message[10:12]
          status = message[12:14]
        case "2E":
          LSB_Tag_ID_MSB = message[4:12]
          Aktor = message[12:14]
          LSB_Tag_ID_MSB = invert_byte_order(LSB_Tag_ID_MSB)
        case "11":
          status = message[4:6]
        case _:
          #implement further cases if needed. results are currently unused but prepared to be usable
          pass
      
def transform_string_to_array(string):
  hex_array = []
  for i in range(0, len(string), 2):
    hex_value = int(string[i:i+2], 16)
    hex_array.append(hex_value)
  return hex_array

def transform_array_to_string(array):
  return ''.join(array)

def invert_byte_order(string:str):
  if not string.isalnum():
    print("Warning: String should be in hexadecimal format")
    return ""
  byte_array = bytearray.fromhex(string)
  byte_array.reverse()
  return transform_array_to_string(byte_array.hex())
        
    
def crc_8(buf:bytearray) -> np.uint8:
  """
  Berechnet die 8-Bit-CRC-Prüfsumme für ein Datenfeld.

  Args:
    buf: Ein Byte-Array mit den Daten.
    data_byte: Die Länge des Datenfeldes in Bytes.

  Returns:
    Die berechnete CRC-8-Prüfsumme.
  """

  crc = np.uint8(0x00)
  data_bit = np.uint8(0x80)
  data_byte = buf[1]-1
  while data_byte > 0:
    if ((crc & 0x01) != 0) != ((buf[data_byte] & data_bit) != 0):
      # Datenbit EXOR mit Rückkopplung
      crc >>= 1
      crc ^= 0xCD
    else:
      crc >>= 1
    data_bit >>= 1

    if not data_bit:
      data_bit = np.uint8(0x80)
      data_byte -= 1

  return crc
# 0x22, 0x06, 0x2E, 0xD0, 0x07, 0xAA, 0x59, 0x03, 0x00, 0xD0, 0x07
# Beispiel 22 06 2E D0 07 AA 59 03 00 D0 07
# really recieved: 0x0d 0x22 0x06 0x26 0xd0 0x07 0xaa 0x59 0x03 0x00 0xd0 0x07 with crc=0xaf
# 22 06 26 D007 765903 00D0072A
# buf = [0x02, 0x0D, 0x22, 0x06, 0x2E, 0xD0, 0x07, 0xAA, 0x59, 0x03, 0x00, 0xD0, 0x07, 0xD0, 0x03]
# buf = [0x02, 0x0E, 0x2E, 0x76, 0x59, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x03]
# buf = [0x02, 0x03, 0x26, 0xfe, 0x03]
if __name__ == "__main__":
  command = DatagramFactory()
  cmd = command.set_tag_actor()
  print(f"Command: {cmd}")
  buf = transform_string_to_array("020326FE03")
  crc = crc_8(buf)
  print(f"CRC-8: {hex(crc)}")
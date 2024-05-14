import socket
from .datagram import DatagramFactory

# Default IP address
DEFAULT_IP = "192.168.0.42"

# Class for TCP connection
class TCPConnection:
    def __init__(self, ip=DEFAULT_IP, port=10001):
        self.ip = ip
        self.port = port
        self.sock = None
        self.datagram_factory = DatagramFactory()

    def __enter__(self):
        # Create and connect socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        # Close socket
        if self.sock is not None:
            self.sock.close()
            
    def send(self, message):
        # Add STX and <CR> to the message
        message_with_stx_cr = '\x02' + message + '\r'

        # Convert message to bytes and send
        self.sock.sendall(message_with_stx_cr.encode())

    def receive(self):
    # Receive until the first ETX byte
        message_data = b''
        while True:
            byte = self.sock.recv(1)
            if byte == b'\x02':  # STX
                continue  # Discard STX and start accumulating
            elif byte == b'\r' or byte == b'\x03':  # ETX
                break
            message_data += byte

        # Handle potential errors
        if not message_data:
            print("Invalid message format: missing or empty content between STX/ETX")
            return b''.decode(), 0
        message = ''
        try:
            message=message_data.decode()
        except UnicodeDecodeError:
            print("Invalid message format: non-ASCII content")
            return b''.decode(), 0
        return message, len(message_data)
    
    # function that sends a message and waits for a response. should return true when the response is recieved with an ack
    def send_and_wait_for_response(self, message):
        self.send(message)
        message_received, _ = self.receive()
        max_tries = 100
        while not (self.__check_acknowledged_response(message, message_received)):
            self.send(message)            
            try:
                message_received, _ = self.receive()
            except Exception as e:
                print(f"Error receiving message: {e}")
                message_received = ""            
            max_tries -= 1
            if max_tries == 0:
                return False
        return True

    def __check_acknowledged_response(self, message, message_received):
        return self.datagram_factory.get_command(message_received) == self.datagram_factory.get_command(message) and self.datagram_factory.is_ack(message_received)
    
def hex_to_decimal(hex_string):
    try:
        decimal_number = int(hex_string, 16)
    except ValueError:
        return 0
    return decimal_number

# Main function
def main():
    # Enter IP address and port
    ip = DEFAULT_IP
    port = 10001
    datagram_factory = DatagramFactory()

    # Connect to server
    with TCPConnection(ip, port) as connection:
        # Create message to send
        # message_to_send = "\x020E2E765903030000000000000065\x03"
        # message_to_send = "0327CF" #Get Software Version
        message_to_send = datagram_factory.set_tag_actor(hex_tag_id = "000359AA", actor = "01")        
        for i in range(0, 5):
            # Receive message
            if connection.send_and_wait_for_response(message_to_send):
                print("Lights on")
        message_to_send = datagram_factory.set_tag_actor(hex_tag_id = "000359AA", actor = "00")
        if connection.send_and_wait_for_response(message_to_send):
            print("Lights off")
            
if __name__ == "__main__":
    main()
    

    
'''
Response Tag Present	STX	n	Source 	22	Ack/Nak	Status Tag	LSB LF ID MSB		LSB Tag ID MSB				LSB HF ID MSB		RSSI X	RSSI Y	RSSI Z	RSSI Abstand	CRC	ETX
Set Tag Actor	STX	n	Target	2E	Ack/Nak	LSB Tag ID MSB				Aktor	tbd	tbd	tbd	tbd	tbd	tbd	tbd	CRC	ETX	
also mache ich für Set Tag Actor sowas wie: 0x02 n 2E 0x06 0x00035976 10000000 crc <CR> 
oder Reader Status	0x02	n 	26	0x06	Output	Modis	Input	LF-Dist.	Status	max.LF-Ampl.		C-kombi	CRC	ETX

0x00035976 -> ASCII:35976


test message:
stx: 0x02
n: 0x0E
cmd: 2E
ID: 0x00035976 -> 765903
Aktor: 0x80 
0x00 00 00 00 00 00 00
CRC: 
ETX: 0x03


unsere default tags:
219510 -> 00035976
219562 -> 000359AA
'''
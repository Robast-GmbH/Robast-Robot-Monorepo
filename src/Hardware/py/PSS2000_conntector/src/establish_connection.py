import socket

# Default IP address
DEFAULT_IP = "10.10.13.10"

# Class for TCP connection
class TCPConnection:
    def __init__(self, ip=DEFAULT_IP, port=10001):
        self.ip = ip
        self.port = port
        self.sock = None

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
        message_with_stx_cr = "\x02" + message + "\x03"

        # Convert message to bytes
        bytes_to_send = message_with_stx_cr.encode()

        # Send byte array
        self.sock.sendall(bytes_to_send)

    def receive(self):
        # Receive STX byte
        stx_byte = self.sock.recv(1)
        if stx_byte != b'\x02':
            print("Invalid message format: missing STX byte")
            return b''.decode(), b''.decode()

        # Receive length byte
        length_byte = self.sock.recv(2)
        
        # Check if any byte of length_byte contains '\x03'
        if b'\x03' in length_byte:
            return b''.decode(), b''.decode()
        
        bytes_length = hex_to_decimal(length_byte.decode())*2

        # Receive byte array
        bytes_received = self.sock.recv(bytes_length)

        # # Receive ETX byte
        # etx_byte = self.sock.recv(1)
        # if etx_byte != b'\x03':
        #     raise ValueError("Invalid message format: missing ETX byte")

        # Convert byte array to string
        return bytes_received.decode(), bytes_length
    
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

    # Connect to server
    with TCPConnection(ip, port) as connection:
        # Create message to send
        # message_to_send = "\x020E2E765903030000000000000065\x03"
        # message_to_send = "0327CF" #Get Software Version
        message_to_send = "0326FE" #Reader Status
        # Send message
        
        for i in range(0, 10):
            # Receive message
            connection.send(message_to_send)
            message_received, length = connection.receive()
            print(f"Received message: {message_received} with length: {length}")
if __name__ == "__main__":
    main()

'''
Response Tag Present	STX	n	Source 	22	Ack/Nak	Status Tag	LSB LF ID MSB		LSB Tag ID MSB				LSB HF ID MSB		RSSI X	RSSI Y	RSSI Z	RSSI Abstand	CRC	ETX
Set Tag Actor	STX	n	Target	2E	Ack/Nak	LSB Tag ID MSB				Aktor	tbd	tbd	tbd	tbd	tbd	tbd	tbd	CRC	ETX	
also mache ich für Set Tag Actor sowas wie: 0x02 n 2E 0x06 0x00035976 10000000 crc <CR> 
oder Reader Status	0x02	n 	26	0x06	Output	Modis	Input	LF-Dist.	Status	max.LF-Ampl.		C-kombi	CRC	ETX

0x00035976 -> ASCII:5976


test message:
stx: 0x02
n: 0x0E
cmd: 2E
ID: 0x00035976 -> 765903
Aktor: 0x80 
0x00 00 00 00 00 00 00
CRC: 
ETX: 0x03
'''
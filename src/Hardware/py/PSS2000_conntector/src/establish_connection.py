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
        # Convert message to bytes
        bytes_to_send = message.encode()

        # Send length of byte array
        self.sock.sendall(len(bytes_to_send).to_bytes(4, byteorder="big"))

        # Send byte array
        self.sock.sendall(bytes_to_send)

    def receive(self) -> bytes:
        # Receive STX byte
        stx_byte = self.sock.recv(1)
        if stx_byte != b'\x02':
            raise ValueError("Invalid message format: missing STX byte")

        # Receive length byte
        length_byte = self.sock.recv(1)
        bytes_length = int.from_bytes(length_byte, byteorder="big")

        # Receive byte array
        bytes_received = self.sock.recv(bytes_length)

        # Receive ETX byte
        # etx_byte = self.sock.recv(1)
        # if etx_byte != b'\x03':
        #     raise ValueError("Invalid message format: missing ETX byte")

        # Convert byte array to string
        return bytes_received

# Main function
def main():
    # Enter IP address and port
    ip = DEFAULT_IP
    port = 10001

    # Connect to server
    with TCPConnection(ip, port) as connection:
        # # Create message to send
        # message_to_send = "Hello, server!"

        # # Send message
        # connection.send(message_to_send)
        
        for i in range(0, 20):
            # Receive message
            message_received = connection.receive()
            # Print received message
            decoded_message = message_received.decode()
            if (len(decoded_message)+1) % 2 != 0:
                decoded_message = '0' + decoded_message
            print("Received message:", decoded_message)
if __name__ == "__main__":
    main()

'''
Response Tag Present	STX	n	Source 	22	Ack/Nak	Status Tag	LSB LF ID MSB		LSB Tag ID MSB				LSB HF ID MSB		RSSI X	RSSI Y	RSSI Z	RSSI Abstand	CRC	ETX
Set Tag Actor	STX	n	Target	2E	Ack/Nak	LSB Tag ID MSB				Aktor	tbd	tbd	tbd	tbd	tbd	tbd	tbd	CRC	ETX	
also mache ich für Set Tag Actor sowas wie: 0x02 n 2E 0x06 0x00035976 10000000 crc <CR> 
oder Reader Status	0x02	n 	26	0x06	Output	Modis	Input	LF-Dist.	Status	max.LF-Ampl.		C-kombi	CRC	ETX

0x00035976 -> ASCII:5976

'''
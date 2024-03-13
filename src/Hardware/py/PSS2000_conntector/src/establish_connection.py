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

    def receive(self):
        # Receive length of byte array
        bytes_length = int.from_bytes(self.sock.recv(4), byteorder="big")

        # Receive byte array
        bytes_received = self.sock.recv(bytes_length)

        # Convert byte array to string
        return bytes_received.decode()

# Main function
def main():
    # Enter IP address and port
    ip = input("IP address: ") or DEFAULT_IP
    port = int(input("Port: ")) or 10001

    # Connect to server
    with TCPConnection(ip, port) as connection:
        # Create message to send
        message_to_send = "Hello, server!"

        # Send message
        connection.send(message_to_send)

        # Receive message
        message_received = connection.receive()

        # Print received message
        print("Received message:", message_received)

if __name__ == "__main__":
    main()

'''
Response Tag Present	STX	n	Source 	22	Ack/Nak	Status Tag	LSB LF ID MSB		LSB Tag ID MSB				LSB HF ID MSB		RSSI X	RSSI Y	RSSI Z	RSSI Abstand	CRC	ETX
Set Tag Actor	STX	n	Target	2E	Ack/Nak	LSB Tag ID MSB				Aktor	tbd	tbd	tbd	tbd	tbd	tbd	tbd	CRC	ETX	
also mache ich für Set Tag Actor sowas wie: 0x02 n 2E 0x06 0x00035976 10000000 crc <CR> 
oder Reader Status	0x02	n 	26	0x06	Output	Modis	Input	LF-Dist.	Status	max.LF-Ampl.		C-kombi	CRC	ETX

0x00035976 -> ASCII:5976
'''
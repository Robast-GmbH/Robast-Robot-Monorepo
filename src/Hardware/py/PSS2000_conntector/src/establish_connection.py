import socket
from bitarray import bitarray

# Default IP address
DEFAULT_IP = "10.10.13.10"

# Class for TCP connection
class TCPConnection:
    def __init__(self, ip=DEFAULT_IP, port=5000):
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

    def send(self, bit_array):
        # Convert bit array to byte array
        bytes_to_send = bit_array.tobytes()

        # Send length of byte array
        self.sock.sendall(len(bytes_to_send).to_bytes(4, byteorder="big"))

        # Send byte array
        self.sock.sendall(bytes_to_send)

    def receive(self):
        # Receive length of byte array
        bytes_length = int.from_bytes(self.sock.recv(4), byteorder="big")

        # Receive byte array
        bytes_received = self.sock.recv(bytes_length)

        # Convert byte array to bit array
        return bitarray(bytes_received)

# Main function
def main():
    # Enter IP address and port
    ip = input("IP address: ") or DEFAULT_IP
    port = int(input("Port: ")) or 5000

    # Connect to server
    with TCPConnection(ip, port) as connection:
        # Create bit array to send
        bit_array_to_send = bitarray([1, 0, 1, 1, 0, 0, 1, 1])

        # Send bit array
        connection.send(bit_array_to_send)

        # Receive bit array
        bit_array_received = connection.receive()

        # Print received bit array
        print("Received bit array:", bit_array_received)

if __name__ == "__main__":
    main()

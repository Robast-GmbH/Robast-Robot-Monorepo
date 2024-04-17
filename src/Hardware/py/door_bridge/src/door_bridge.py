#TODO @TAlscher: fix imports
from src.PSS2000_connector.src.datagram import DatagramFactory
from src.PSS2000_connector.src.establish_connection import TCPConnection
import time

#python3 -m src.door_bridge.src.door_bridge


def convert_to_hex(decimal_string):
    hex_string = hex(int(decimal_string))[2:].upper()
    return hex_string.zfill(8)

class DoorBridge:
    def __init__(self):
        self.datagram_factory = DatagramFactory()
        self.ip = "10.10.13.10"
        self.port = 10001
        self.door_is_open_state = False

    def open_door(self, tag_id = "219562"):
        hex_tag_id = convert_to_hex(tag_id)
        with TCPConnection(self.ip, self.port) as connection:
            message_to_send = self.datagram_factory.set_tag_actor(hex_tag_id = hex_tag_id, actor = "01")
            self.door_is_open_state = connection.send_and_wait_for_response(message_to_send)
            return self.door_is_open_state
        return False

    def close_door(self, tag_id = "219562"):
        hex_tag_id = convert_to_hex(tag_id)
        with TCPConnection(self.ip, self.port) as connection:
            message_to_send = self.datagram_factory.set_tag_actor(hex_tag_id = hex_tag_id, actor = "00")
            self.door_is_open_state = connection.send_and_wait_for_response(message_to_send)
            return self.door_is_open_state
        return False
    
    #do we need a funtion that displays all tags in range?
        
if __name__ == "__main__":
    DoorBridge().open_door()
    time.sleep(5)
    DoorBridge().close_door()
import unittest
from error_utils_py.generic_error_converter import message_to_string, string_to_message
from communication_interfaces.msg import DrawerAddress

class TestGenericErrorConverter(unittest.TestCase):
    def test_message_to_string_and_string_to_message(self):
        # Create a test message
        msg_to_be_serialized = DrawerAddress()
        msg_to_be_serialized.module_id = 12
        msg_to_be_serialized.drawer_id = 3

        wrong_msg = DrawerAddress()
        wrong_msg.module_id = 12
        wrong_msg.drawer_id = 4

        serialized_str = message_to_string(msg_to_be_serialized)

        deserialized_msg = string_to_message(serialized_str)

        self.assertEqual(msg_to_be_serialized, deserialized_msg)
        self.assertNotEqual(wrong_msg, deserialized_msg)

if __name__ == '__main__':
    unittest.main()
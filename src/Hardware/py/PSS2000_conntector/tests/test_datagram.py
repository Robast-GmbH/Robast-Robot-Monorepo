import unittest
from PSS2000_connector.datagram import DatagramFactory

class TestDatagramFactory(unittest.TestCase):
    def setUp(self):
        self.factory = DatagramFactory()

    def test_set_tag_actor(self):
        # Test valid tag ID and actor
        expected_command = "0F2Eaa5903000100000000000000E0"
        command = self.factory.set_tag_actor(hex_tag_id="000359AA", actor="01")
        self.assertEqual(command, expected_command)

        # Test invalid tag ID (not 4 bytes)
        with self.assertRaises(ValueError):
            self.factory.set_tag_actor(hex_tag_id="123", actor="01")

    def test_get_software_version(self):
        expected_command = "0327CF"
        command = self.factory.get_software_version()
        self.assertEqual(command, expected_command)

    def test_get_status(self):
        expected_command = "0326FE"
        command = self.factory.get_status()
        self.assertEqual(command, expected_command)

    def test_get_command(self):
        message = "ABCD1234"
        expected_command = "CD"
        command = self.factory.get_command(message)
        self.assertEqual(command, expected_command)

    def test_is_ack(self):
        # Test ACK message
        message = "ABCD0600"
        self.assertTrue(self.factory.is_ack(message))

        # Test NAK message
        message = "ABCD1500"
        self.assertFalse(self.factory.is_ack(message))

if __name__ == '__main__':
    unittest.main()
import unittest
import os
from middleware.user_system.user_repository import UserRepository


# has to be run from middleware directory because of the relative import
class TestUserSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.db_name = "test_users.db"
        cls.user_system = UserRepository(cls.db_name)

    @classmethod
    def tearDownClass(cls):
        os.remove(cls.db_name)

    def test_create_user(self):
        user = self.user_system.create_user("nfc123", "John", "Doe", ["staff"])
        self.assertIsNotNone(user)
        if user is not None:
            self.assertEqual(user.first_name, "John")
            self.assertEqual(user.last_name, "Doe")
            self.assertIn("staff", user.user_groups)

    def test_get_user(self):
        user = self.user_system.create_user("nfc456", "Jane", "Doe", ["admin"])
        if user is not None:
            retrieved_user = self.user_system.get_user(user.user_id)
            self.assertIsNotNone(retrieved_user)
            if retrieved_user is not None:
                self.assertEqual(retrieved_user.first_name, "Jane")
                self.assertEqual(retrieved_user.last_name, "Doe")
                self.assertIn("admin", retrieved_user.user_groups)

    def test_update_user(self):
        user = self.user_system.create_user("nfc789", "Alice", "Smith", ["patient"])
        if user is not None:
            self.user_system.update_user(
                user.user_id, first_name="Alicia", user_groups=["staff"]
            )
            updated_user = self.user_system.get_user(user.user_id)
            if updated_user is not None:
                self.assertEqual(updated_user.first_name, "Alicia")
                self.assertIn("staff", updated_user.user_groups)
                self.assertNotIn("patient", updated_user.user_groups)

    def test_delete_user(self):
        user = self.user_system.create_user("nfc000", "Bob", "Brown", ["staff"])
        if user is not None:
            self.user_system.delete_user(user.user_id)
            deleted_user = self.user_system.get_user(user.user_id)
            self.assertIsNone(deleted_user)

    def test_auth_user(self):
        user = self.user_system.create_user("nfc001", "Charlie", "Chaplin", ["admin"])
        if user is not None:
            auth_result = self.user_system.auth_user(user.user_id, "nfc001")
            self.assertTrue(auth_result)
            auth_result_fail = self.user_system.auth_user(user.user_id, "wrong_nfc")
            self.assertFalse(auth_result_fail)


if __name__ == "__main__":
    unittest.main()

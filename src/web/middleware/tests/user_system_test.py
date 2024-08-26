import unittest
import os
from user_system.user_repository import UserRepository


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
        user = self.user_system.create_user(
            "Dr.", "Bob", "Brown", "some_station", "some_room", ["staff"]
        )
        self.assertIsNotNone(user)
        if user is not None:
            self.assertEqual(user.first_name, "Bob")
            self.assertEqual(user.last_name, "Brown")
            self.assertIn("staff", user.user_groups)

    def test_get_user(self):
        user = self.user_system.create_user(
            "Dr.", "Bob", "Brown", "some_station", "some_room", ["staff"]
        )
        if user is not None:
            retrieved_user = self.user_system.get_user(user.id)
            self.assertIsNotNone(retrieved_user)
            if retrieved_user is not None:
                self.assertEqual(retrieved_user.first_name, "Bob")
                self.assertEqual(retrieved_user.last_name, "Brown")
                self.assertIn("staff", retrieved_user.user_groups)

    def test_update_user(self):
        user = self.user_system.create_user(
            "Dr.", "Bob", "Brown", "some_station", "some_room", ["staff"]
        )
        if user is not None:
            self.user_system.update_user(
                user.id, first_name="Alicia", user_groups=["staff"]
            )
            updated_user = self.user_system.get_user(user.id)
            if updated_user is not None:
                self.assertEqual(updated_user.first_name, "Alicia")
                self.assertIn("staff", updated_user.user_groups)
                self.assertNotIn("patient", updated_user.user_groups)

    def test_delete_user(self):
        user = self.user_system.create_user(
            "Dr.", "Bob", "Brown", "some_station", "some_room", ["staff"]
        )
        if user is not None:
            self.user_system.delete_user(user.id)
            deleted_user = self.user_system.get_user(user.id)
            self.assertIsNone(deleted_user)


if __name__ == "__main__":
    unittest.main()

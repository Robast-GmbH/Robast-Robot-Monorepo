#include "catch.hpp"
#include "fakeit.hpp"
#include "test/test_nfc_bridge.hpp"

namespace nfc_bridge
{

  SCENARIO("Key-validaten Test")
  {
    GIVEN(
        "The Serial reader gets mocked and initiated in the NFC bridge Node and there is a list with possible keys, "
        "which could be the read key")
    {
      rclcpp::init(0, nullptr);
      fakeit::Mock<serial_helper::MockSerialHelper> Serial_helper_mock;
      Method(Serial_helper_mock, open_serial) = "";
      fakeit::Fake(Method(Serial_helper_mock, close_serial));
      Method(Serial_helper_mock, ascii_interaction);
      Method(Serial_helper_mock, send_ascii_cmd) = "";
      fakeit::Mock<db_helper::MockPostgreSqlHelper> db_helper_mock;
      fakeit::When(Method(db_helper_mock, perform_query))
          .AlwaysDo(
              [=](std::string sqlStatment, std::shared_ptr<std::string>) -> std::vector<std::vector<std::string>>
              {
                std::vector<std::vector<std::string>> result;
                result.push_back(std::vector<std::string>({"name"}));
                if (sqlStatment.find("12bab1cc5b867068c127a6c8299e3f61") != std::string::npos)
                {
                  result.push_back(std::vector<std::string>({"Test User1"}));
                }
                else
                {
                  result.push_back(std::vector<std::string>({""}));
                }
                return result;
              });

      fakeit::When(Method(db_helper_mock, checkUserTag))
          .AlwaysDo(
              [=](std::string sqlStatment,
                  std::shared_ptr<std::string> result_data,
                  std::shared_ptr<int>,
                  std::shared_ptr<std::string>,
                  std::vector<std::string>) -> bool
              {
                if (sqlStatment.find("12bab1cc5b867068c127a6c8299e3f61") != std::string::npos)
                {
                  *result_data = "Test User1";
                  return true;
                }
                else
                {
                  *result_data = "";
                  return false;
                }
              });
      std::string no_key_found = "";
      TestNFCBridge* nfc_reader = new TestNFCBridge(&Serial_helper_mock.get(), &db_helper_mock.get());
      std::string target_user = "Test User1";
      WHEN("Key is on the List")
      {
        std::string valid_key = "12bab1cc5b867068c127a6c8299e3f61";

        THEN("The result of the validation should be the key entered as it was found.")
        {
          std::string result_name;
          int result_id;
          REQUIRE(nfc_reader->lookup_user_tag(
              valid_key, std::make_shared<std::string>(result_name), std::make_shared<int>(result_id)));
          REQUIRE(result_name == target_user);
        }
      }

      WHEN("Key is not in the List")
      {
        std::string invalid_key = "11111111111111111111111111111111";

        THEN("The result of the validation should be an empty string as it is not a valid key")
        {
          std::string result_name;
          int result_id;
          REQUIRE_FALSE(nfc_reader->lookup_user_tag(
              invalid_key, std::make_shared<std::string>(result_name), std::make_shared<int>(result_id)));
          REQUIRE(result_name == no_key_found);
        }
      }

      WHEN("Key is empty string")
      {
        std::string empty_key = "";

        THEN("The result of the validation should be an empty string as it is not a valid key")
        {
          std::string result_name;
          int result_id;
          REQUIRE_FALSE(nfc_reader->lookup_user_tag(
              empty_key, std::make_shared<std::string>(result_name), std::make_shared<int>(result_id)));
          REQUIRE(result_name == no_key_found);
        }
      }
      rclcpp::shutdown();
    }
  }

  SCENARIO("Card Reading Test")
  {
    GIVEN("The Serial reader gets mocked and initiated in the NFC bridge Node.")
    {
      rclcpp::init(0, nullptr);
      fakeit::Mock<serial_helper::MockSerialHelper> serial_helper_mock;
      Method(serial_helper_mock, open_serial) = "";
      fakeit::Fake(Method(serial_helper_mock, close_serial));
      Method(serial_helper_mock, read_serial);
      fakeit::Mock<db_helper::MockPostgreSqlHelper> db_helper_mock;
      TestNFCBridge* nfc_reader = new TestNFCBridge(&serial_helper_mock.get(), &db_helper_mock.get());

      WHEN("No card is detected")
      {
        fakeit::When(Method(serial_helper_mock, read_serial))
            .AlwaysDo(
                [=](auto& responce, auto responce_size) -> int
                {
                  *responce = "";
                  responce_size = 0;
                  return 0;
                });

        THEN("The reader Returns a empty string and indicates that nothing was found")
        {
          std::string key;
          REQUIRE_FALSE(nfc_reader->read_nfc_code(std::make_shared<std::string>(key)));
          REQUIRE(key == "");
        }
      }
      // TODO Torben: valid again after RE-1783
      // WHEN("The key from the card is empty")
      // {
      //   std::string key_from_card = "";
      //   fakeit::When(Method(serial_helper_mock, ascii_interaction))
      //       .AlwaysDo(
      //           [=](auto cmd, auto& responce, auto responce_size) -> std::string
      //           {
      //             (void) responce_size;
      //             if (cmd == NFC_READ_MC("02"))
      //             {
      //               return *responce = key_from_card;
      //             }
      //             return *responce = "0001";
      //           });

      //   THEN("The reader returns the scanned key which is empty and indicates that something was found.")
      //   {
      //     std::string key;
      //     REQUIRE(nfc_reader->scan_tag(std::make_shared<std::string>(key)));
      //     REQUIRE(key == key_from_card);
      //   }
      // }

      // TODO Torben: valid again after RE-1783
      //      WHEN("The key from the card is correct formated")
      //      {
      //        std::string key_from_card = "6df1933415dde9fa1f9f6998a26b40db";
      //        fakeit::When(Method(serial_helper_mock, ascii_interaction))
      //            .AlwaysDo(
      //                [=](auto cmd, auto& responce, auto responce_size) -> std::string
      //                {
      //                  (void) responce_size;
      //                  if (cmd == NFC_READ_MC("02"))
      //                  {
      //                    return *responce = key_from_card;
      //                  }
      //                  return *responce = "0001";
      //                });

      //       THEN(
      //           "The reader returns the scanned key, indecates that a Key has been found and on the serial connection
      //           the " "a NFC login message followed by a Read Message was send in the reading process to get the data
      //           from the " "card")
      //       {
      //         std::string key;
      //         REQUIRE(nfc_reader->scan_tag(std::make_shared<std::string>(key)));
      //         REQUIRE(key == key_from_card);
      //         REQUIRE(fakeit::Verify(
      //             Method(serial_helper_mock, ascii_interaction).Using(NFC_LOGIN_MC_STANDARD("00"), fakeit::_,
      //             fakeit::_), Method(serial_helper_mock, ascii_interaction).Using(NFC_READ_MC("02"), fakeit::_,
      //             fakeit::_)));
      //       }
      //    }
    }
    rclcpp::shutdown();
  }

  SCENARIO("Full NFC reader process Test")
  {
    GIVEN("the mocked reader gets used in the nfc bridge with a list of possible keys")
    {
      rclcpp::init(0, nullptr);
      fakeit::Mock<serial_helper::MockSerialHelper> serial_helper_mock;
      Method(serial_helper_mock, open_serial) = "";
      fakeit::Fake(Method(serial_helper_mock, close_serial));
      Method(serial_helper_mock, ascii_interaction);
      Method(serial_helper_mock, send_ascii_cmd) = "";
      fakeit::Mock<db_helper::MockPostgreSqlHelper> db_helper_mock;

      fakeit::When(Method(db_helper_mock, checkUserTag))
          .AlwaysDo(
              [=](std::string sqlStatment,
                  std::shared_ptr<std::string> result_data_username,
                  std::shared_ptr<int>,
                  std::shared_ptr<std::string>,
                  std::vector<std::string>) -> bool
              {
                if (sqlStatment.find("6df1933415dde9fa1f9f6998a26b40db") != std::string::npos)
                {
                  *result_data_username = "Test User1";
                  return true;
                }
                else
                {
                  *result_data_username = "";
                  return false;
                }
              });

      std::string target_user = "Test User1";
      TestNFCBridge nfc_reader = TestNFCBridge(&serial_helper_mock.get(), &db_helper_mock.get());
      std::vector<std::string> list{"Test User1", "Test User2", "Test User3", "Test User4", "Test User5"};
      std::string no_key_found = "";

      WHEN("The key from the card is in the list of possible keys")
      {
        std::string key_from_card = "6df1933415dde9fa1f9f6998a26b40db";
        fakeit::When(Method(serial_helper_mock, ascii_interaction))
            .AlwaysDo(
                [=](auto cmd, auto& responce, auto responce_size) -> std::string
                {
                  (void) responce_size;
                  (void) cmd;
                  return *responce = key_from_card;
                });

        THEN(
            "The reader returns the scanned key  and a NFC login message followed by a Read Message should have been "
            "performed to read data from the Card")
        {
          std::string key;
          REQUIRE(nfc_reader.read_nfc_code(std::make_shared<std::string>(key)));
          REQUIRE(key == target_user);
        }
      }

      WHEN("The key from the card is not in the list of possible keys")
      {
        std::string key_from_card = "";
        fakeit::When(Method(serial_helper_mock, ascii_interaction))
            .AlwaysDo(
                [=](auto cmd, auto& responce, auto responce_size) -> std::string
                {
                  (void) responce_size;
                  (void) cmd;
                  return *responce = key_from_card;
                });

        THEN("The reader returns an empty sting")
        {
          std::string key;
          REQUIRE(nfc_reader.read_nfc_code(std::make_shared<std::string>(key)));
          REQUIRE(key == no_key_found);
        }
      }

      rclcpp::shutdown();
    }
  }
}   // namespace nfc_bridge
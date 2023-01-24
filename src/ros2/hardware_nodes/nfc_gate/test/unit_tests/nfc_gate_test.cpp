#include "catch.hpp"
#include "fakeit.hpp"
#include "test/test_nfc_gate.hpp"


using namespace fakeit;
namespace robast
{

  SCENARIO("Key-validaten Test")
  {
    GIVEN("The Serial reader gets mocked and initiated in the NFC gate Node and there is a list with possible keys, which could be the read key")
    {
      rclcpp::init(0, nullptr);
      Mock<serial_helper::ISerialHelper> Serial_helper_mock;
      Method(Serial_helper_mock, open_serial) = "";
      Fake(Method(Serial_helper_mock, close_serial));
      Method(Serial_helper_mock, ascii_interaction);
      Method(Serial_helper_mock, send_ascii_cmd) = "";
      Mock<db::IDBHelper> db_helper_mock;
      Method(db_helper_mock, open_connection) = "";
      Method(db_helper_mock, close_connection);
      When(Method(db_helper_mock, perform_query)).AlwaysDo([=](std::string sqlStatment, std::vector< std::vector<std::string> >*result_data, std::vector<std::string> *result_header) -> bool
        {
          if (sqlStatment.find("12bab1cc5b867068c127a6c8299e3f61") != string::npos)
          {
            result_data->push_back(std::vector<string>{"Test User1"});
            result_header->push_back("name");
            return true;
          }
          else
          {
            result_header->push_back("name");
            return false;
          }
        });
       When(Method(db_helper_mock, checkUserTag)).AlwaysDo([=](std::string sqlStatment, std::vector<std::string> Lookup_scope, std::string*result_data) -> bool
        {
          if (sqlStatment.find("12bab1cc5b867068c127a6c8299e3f61") != string::npos)
          {
            *result_data ="Test User1";
            return true;
          }
          else
          {
            *result_data ="";
            return false;
          }
        });
      string no_key_found = "";
      TestNFCGate* nfc_reader = new TestNFCGate(&Serial_helper_mock.get(), &db_helper_mock.get());
      std::vector<std::string> list{ "Test User1", "Test User2", "Test User3", "Test User4", "Test User5" };
      string target_user = "Test User1";
      WHEN("Key is on the List")
      {
        string valid_key = "12bab1cc5b867068c127a6c8299e3f61";
       

        THEN("The result of the validation should be the key entered as it was found.")
        {
          bool found;
          string result = nfc_reader->validate_key(valid_key, list, &found);
          CHECK(found);
          REQUIRE(result == target_user);
        }
      }

      WHEN("Key is not in the List")
      {
        string invalid_key = "11111111111111111111111111111111";

        THEN("The result of the validation should be an empty string as it is not a valid key")
        {
          bool found;
          string result = nfc_reader->validate_key(invalid_key, list, &found);
          CHECK_FALSE(found);
          REQUIRE(result == no_key_found);
          
        }
      }

      WHEN("Key is empty string")
      {
        string empty_key = "";

        THEN("The result of the validation should be an empty string as it is not a valid key")
        {
          bool found;
          string result = nfc_reader->validate_key(empty_key, list, &found);
          CHECK_FALSE(found);
          REQUIRE(result == no_key_found);

        }
      }
      rclcpp::shutdown();
    }
  }

  SCENARIO("Card Reading Test")
  {
    GIVEN("The Serial reader gets mocked and initiated in the NFC gate Node.")
    {
      rclcpp::init(0, nullptr);
      Mock<serial_helper::ISerialHelper> serial_helper_mock;
      Method(serial_helper_mock, open_serial) = "";
      Fake(Method(serial_helper_mock, close_serial));
      Method(serial_helper_mock, ascii_interaction);
      Method(serial_helper_mock, send_ascii_cmd) = "";
      bool found;
      Mock<db::IDBHelper> db_helper_mock;
      TestNFCGate* nfc_reader = new TestNFCGate(&serial_helper_mock.get(), &db_helper_mock.get());


      WHEN("No card is detected")
      {
        When(Method(serial_helper_mock, ascii_interaction)).AlwaysDo([=](auto cmd, auto& responce, auto responce_size)-> string
          {
            (void)responce_size;
            (void)cmd;
            return *responce = "0000";
          });

        THEN("The reader Returns a empty string and indicates that nothing was found")
        {

          string key = nfc_reader->scan_tag(&found);
          CHECK(key == "");
          REQUIRE_FALSE(found);
        }
      }

      WHEN("The key from the card is empty")
      {
        string key_from_card = "";
        When(Method(serial_helper_mock, ascii_interaction)).AlwaysDo([=](auto cmd, auto& responce, auto responce_size)-> string
          {
            (void)responce_size;
            if (cmd == NFC_READ_MC("02"))
            {

              return *responce = key_from_card;
            }
            return *responce = "0001";
          });

        THEN("The reader returns the scanned key which is empty and indicates that something was found.")
        {
          string key = nfc_reader->scan_tag(&found);
          CHECK(key == key_from_card);
          REQUIRE(found);
        }
      }

      WHEN("The key from the card is correct formated")
      {

        string key_from_card = "6df1933415dde9fa1f9f6998a26b40db";
        When(Method(serial_helper_mock, ascii_interaction)).AlwaysDo([=](auto cmd, auto& responce, auto responce_size)-> string
          {
            (void)responce_size;
            if (cmd == NFC_READ_MC("02"))
            {

              return *responce = key_from_card;
            }
            return *responce = "0001";
          });

        THEN("The reader returns the scanned key, indecates that a Key has been found and on the serial connection the a NFC login message followed by a Read Message was send in the reading process to get the data from the card")
        {
          bool found;
          string key = nfc_reader->scan_tag(&found);
          CHECK(found);
          CHECK(key == key_from_card);
          REQUIRE(Verify(Method(serial_helper_mock, ascii_interaction).Using(NFC_LOGIN_MC_STANDART("00"), _, _), Method(serial_helper_mock, ascii_interaction).Using(NFC_READ_MC("02"), _, _)));
        }
      }

    }
    rclcpp::shutdown();
  }

  SCENARIO("Full NFC reader process Test")
  {
    GIVEN("the mocked reader gets used in the nfc gate with a list of possible keys")
    {
      rclcpp::init(0, nullptr);
      Mock<serial_helper::ISerialHelper> serial_helper_mock;
      Method(serial_helper_mock, open_serial) = "";
      Fake(Method(serial_helper_mock, close_serial));
      Method(serial_helper_mock, ascii_interaction);
      Method(serial_helper_mock, send_ascii_cmd) = "";
      Mock<db::IDBHelper> db_helper_mock;
     
      When(Method(db_helper_mock, checkUserTag)).AlwaysDo([=](std::string sqlStatment, std::vector<std::string> Lookup_scope, std::string*result_data) -> bool
        {
          if (sqlStatment.find("6df1933415dde9fa1f9f6998a26b40db") != string::npos)
          {
            *result_data ="Test User1";
            return true;
          }
          else
          {
            *result_data ="";
            return false;
          }
        });
      
      string target_user = "Test User1";
      TestNFCGate* nfc_reader = new TestNFCGate(&serial_helper_mock.get(), &db_helper_mock.get());
      std::vector<std::string> list{ "Test User1", "Test User2", "Test User3", "Test User4", "Test User5" };
      string no_key_found = "";

      WHEN("The key from the card is in the list of possible keys")
      {
        string key_from_card = "6df1933415dde9fa1f9f6998a26b40db";
        When(Method(serial_helper_mock, ascii_interaction)).AlwaysDo([=](auto cmd, auto& responce, auto responce_size)-> string
          {
            (void)responce_size;
            if (cmd == NFC_READ_MC("02"))
            {
              return *responce = key_from_card;
            }
            else if (cmd == DEVICE_STATE)
            {
              return *responce = RESPONCE_DEVICE_STATE_CONFIGURED;
            }
            return *responce = "0001";
          });

        THEN("The reader returns the scanned key  and a NFC login message followed by a Read Message should have been performed to read data from the Card")
        {
          bool found;
          string key = nfc_reader->execute_scan(list, &found);
          REQUIRE(key == target_user);
        }
      }

      WHEN("The key from the card is not in the list of possible keys")
      {
        string key_from_card = "";
        When(Method(serial_helper_mock, ascii_interaction)).AlwaysDo([=](auto cmd, auto& responce, auto responce_size)-> string
          {
            (void)responce_size;
            (void)cmd;
            if (cmd == NFC_READ_MC("02"))
            {
              return *responce = key_from_card;
            }
            else if (cmd == DEVICE_STATE)
            {
              return *responce = RESPONCE_DEVICE_STATE_CONFIGURED;
            }
            return *responce = "0002";
          });

        THEN("The reader returns an empty sting")
        {
          bool found;
          string key = nfc_reader->execute_scan(list, &found);
          REQUIRE(key == no_key_found);
        }
      }

      rclcpp::shutdown();
    }
  }
}
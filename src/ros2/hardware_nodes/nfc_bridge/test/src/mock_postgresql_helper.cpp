#include "test/mock_postgresql_helper.hpp"

namespace db_helper
{
  MockPostgreSqlHelper::MockPostgreSqlHelper(std::map<std::string, std::pair<int, std::string>> valid_user_list)
  {
    id_by_name_of_valid_db_user = valid_user_list;
  }

  bool MockPostgreSqlHelper::checkUserTag(std::string scanned_key,
                      std::shared_ptr<std::string> related_username,
                      std::shared_ptr<int> id,
                      std::shared_ptr<std::string> ,
                      std::vector<std::string>)
  {
    if (id_by_name_of_valid_db_user.find(scanned_key) != id_by_name_of_valid_db_user.end())
    {
      std::pair<int, std::string> found_user = id_by_name_of_valid_db_user[scanned_key];
      *related_username = found_user.second;
      *id = found_user.first;

      return true;
    }
    return false;
  }

  std::string MockPostgreSqlHelper::test_connection()
  {
    return "Dummy";
  }

  bool MockPostgreSqlHelper::createNfcCode(std::string, std::string)
  {
    return true;
  }

  int MockPostgreSqlHelper::createUser(std::string, std::string)
  {
    return -1;
  }

  bool MockPostgreSqlHelper::checkUser(int, std::shared_ptr<std::string>)
  {
    return false;
  }

  std::vector<std::vector<std::string>> MockPostgreSqlHelper::perform_query(std::string, std::shared_ptr<std::string>)
  {
    return std::vector<std::vector<std::string>>();
  }

}   // namespace db_helper
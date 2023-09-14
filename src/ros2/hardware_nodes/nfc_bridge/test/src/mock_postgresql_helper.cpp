#include "test/mock_postgresql_helper.hpp"

namespace db_helper
{
  MockPostgreSqlHelper::MockPostgreSqlHelper(std::map<std::string, std::pair<int,std::string>> valid_user_list)
  {
    user_list_ = valid_user_list;
  }

  bool MockPostgreSqlHelper::checkUserTag(std::string scanned_key,
                                          std::vector<std::string> lookup_scope __attribute__((unused)),
                                          std::shared_ptr<std::string> related_username,
                                          std::shared_ptr<int> id)
  {

    if (user_list_.find(scanned_key) != user_list_.end())
    {
        std::pair<int, std::string> found_user=user_list_[scanned_key];
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

  int MockPostgreSqlHelper::createNfcCode(std::string user_id, int max_id)
  {
    return 222;
  }

  std::string MockPostgreSqlHelper::createUser(std::string first_name, std::string last_name)
  {
    return "Dummy";
  }

  bool MockPostgreSqlHelper::checkUser(std::string id, std::string first_name, std::string last_name)
  {
    return false;
  }

  std::vector<std::vector<std::string>> MockPostgreSqlHelper::perform_query(std::string sql_statment)
  {
    return std::vector<std::vector<std::string>>();
  }

}   // namespace db_helper
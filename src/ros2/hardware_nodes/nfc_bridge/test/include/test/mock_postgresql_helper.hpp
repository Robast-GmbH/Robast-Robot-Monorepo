#ifndef HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_
#define HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "db_helper/i_db_helper.h"

namespace db_helper
{
  class MockPostgreSqlHelper : public IDBHelper
  {
   public:
    MockPostgreSqlHelper(std::map<std::string, std::string> validUserList);

    std::string test_connection();
    std::string createUser(std::string first_name, std::string last_name);
    int createNfcCode(std::string user_id, int max_ID);
    bool checkUser(std::string id, std::string first_name, std::string last_name);
    bool checkUserTag(std::string tag,
                      std::vector<std::string> lookup_scope,
                      std::shared_ptr<std::string> user_name,
                      std::shared_ptr<int> id);
    std::vector<std::vector<std::string>> perform_query(std::string sql_statment);

   private:
    std::map<std::string, std::string> user_list_;
  };
}   // namespace db_helper

#endif /* HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_ */
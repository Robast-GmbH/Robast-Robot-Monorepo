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
    MockPostgreSqlHelper(std::map<std::string, std::pair<int, std::string>> validUserList);

    std::string test_connection();
    int createUser(std::string first_name, std::string last_name);
    bool createNfcCode(std::string user_id, std::string card_id);
    bool checkUser(int id, std::shared_ptr<std::string> error_msg);
    bool checkUserTag(std::string tag,
                      std::vector<std::string> lookup_scope,
                      std::shared_ptr<std::string> user_name,
                      std::shared_ptr<int> id,
                      std::shared_ptr<std::string> error_msg);
    std::vector<std::vector<std::string>> perform_query(std::string sql_statment,
                                                        std::shared_ptr<std::string> error_message);

   private:
    std::map<std::string, std::pair<int, std::string>> user_list_;
  };
}   // namespace db_helper

#endif /* HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_ */
#ifndef HARDWARE_NODES__I_DB_HELPER_H_
#define HARDWARE_NODES__I_DB_HELPER_H_

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace db_helper
{
  class IDBHelper
  {
   public:
    virtual std::string test_connection() = 0;

    virtual int createUser(std::string first_name, std::string last_name) = 0;
    virtual bool createNfcCode(std::string user_id, std::string card_uid) = 0;
    virtual bool checkUser(int id, std::shared_ptr<std::string> error_msg) = 0;
    virtual std::vector<std::vector<std::string>> perform_query(std::string sql_statment, std::shared_ptr<std::string> error_message) = 0;
    virtual bool checkUserTag(std::string tag,
                              std::shared_ptr<std::string> user_name,
                              std::shared_ptr<int> id,
                              std::shared_ptr<std::string> error_msg,
                              std::vector<std::string> lookup_scope=  std::vector<std::string>()) = 0;
  };
}   // namespace db_helper

#endif /* HARDWARE_NODES__I_DB_HELPER_H_ */